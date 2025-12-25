"""
Embedding Pipeline for Docusaurus Documentation

This script extracts text from deployed Docusaurus URLs, generates embeddings using Cohere,
and stores them in Qdrant for RAG-based retrieval.
"""

import os
import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin, urlparse
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv
import time
import logging
from typing import List, Dict, Any
import uuid
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create session with retry strategy
def create_session_with_retries():
    session = requests.Session()
    retry_strategy = Retry(
        total=3,  # Total number of retries
        backoff_factor=1,  # Wait time between retries (1s, 2s, 4s...)
        status_forcelist=[429, 500, 502, 503, 504],  # HTTP status codes to retry
        allowed_methods=["HEAD", "GET", "OPTIONS"]  # HTTP methods to retry
    )
    adapter = HTTPAdapter(max_retries=retry_strategy)
    session.mount("http://", adapter)
    session.mount("https://", adapter)
    return session

session = create_session_with_retries()

# Initialize clients
co = cohere.Client(os.getenv("COHERE_API_KEY"))
qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
qdrant_api_key = os.getenv("QDRANT_API_KEY")
if qdrant_api_key:
    qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
else:
    qdrant_client = QdrantClient(url=qdrant_url)


def get_all_urls(base_url: str) -> List[str]:
    """
    Discover all URLs from the target Docusaurus site.

    Args:
        base_url: The base URL of the Docusaurus site

    Returns:
        List of URLs found on the site
    """
    logger.info(f"Discovering URLs from {base_url}")
    urls = set()
    visited = set()

    # Add the base URL to start with
    urls.add(base_url)

    # Try to get the sitemap first
    sitemap_url = urljoin(base_url, "sitemap.xml")
    try:
        response = session.get(sitemap_url)
        if response.status_code == 200:
            # Parse sitemap for URLs
            soup = BeautifulSoup(response.content, 'xml')
            for loc in soup.find_all('loc'):
                url = loc.text.strip()
                if url.startswith(base_url):
                    urls.add(url)
    except Exception as e:
        logger.warning(f"Could not fetch sitemap: {e}")

    # If no sitemap or minimal sitemap, try crawling navigation links
    # For Docusaurus sites, we can often find navigation in the main page
    try:
        response = session.get(base_url)
        response.raise_for_status()
        soup = BeautifulSoup(response.content, 'html.parser')

        # Look for navigation links in common Docusaurus elements
        nav_links = soup.find_all('a', href=True)
        for link in nav_links:
            href = link['href']
            full_url = urljoin(base_url, href)

            # Only add URLs from the same domain
            if urlparse(full_url).netloc == urlparse(base_url).netloc:
                # Filter out non-documentation URLs
                if not any(skip in full_url for skip in ['#', 'mailto:', 'tel:']):
                    urls.add(full_url)

        # Also check for common Docusaurus documentation patterns
        doc_patterns = ['/docs', '/category', '/api']
        for pattern in doc_patterns:
            doc_url = urljoin(base_url, pattern)
            try:
                doc_response = session.get(doc_url)
                if doc_response.status_code == 200:
                    doc_soup = BeautifulSoup(doc_response.content, 'html.parser')
                    doc_links = doc_soup.find_all('a', href=True)
                    for link in doc_links:
                        href = link['href']
                        full_url = urljoin(base_url, href)
                        if urlparse(full_url).netloc == urlparse(base_url).netloc:
                            if not any(skip in full_url for skip in ['#', 'mailto:', 'tel:']):
                                urls.add(full_url)
            except:
                continue

    except Exception as e:
        logger.error(f"Error crawling {base_url}: {e}")

    logger.info(f"Discovered {len(urls)} URLs")
    return list(urls)


def extract_text_from_url(url: str) -> Dict[str, Any]:
    """
    Extract clean text from a given URL.

    Args:
        url: The URL to extract text from

    Returns:
        Dictionary containing the extracted content, title, and metadata
    """
    logger.info(f"Extracting text from {url}")
    try:
        response = session.get(url)
        response.raise_for_status()

        soup = BeautifulSoup(response.content, 'html.parser')

        # Remove script and style elements
        for script in soup(["script", "style", "nav", "header", "footer", "aside"]):
            script.decompose()

        # Try to find the main content area (common selectors for Docusaurus)
        content_selectors = [
            'main',  # Most common
            '[role="main"]',
            '.main-wrapper',  # Common in Docusaurus
            '.theme-doc-markdown',  # Docusaurus documentation pages
            '.markdown',  # Markdown content areas
            '.container',  # General container
            'article',  # Article content
        ]

        content = None
        for selector in content_selectors:
            content_elem = soup.select_one(selector)
            if content_elem:
                content = content_elem
                break

        # If no specific content area found, use body
        if content is None:
            content = soup.find('body')

        if content:
            # Extract text and clean it up
            text = content.get_text(separator=' ', strip=True)
            # Clean up multiple spaces and newlines
            import re
            text = re.sub(r'\s+', ' ', text).strip()
        else:
            # If no content found, use the entire body text
            text = soup.get_text(separator=' ', strip=True)
            text = re.sub(r'\s+', ' ', text).strip()

        # Get page title
        title_tag = soup.find('title')
        title = title_tag.get_text().strip() if title_tag else urlparse(url).path.split('/')[-1] or 'Untitled'

        # Extract any h1 as a more specific title
        h1_tag = soup.find('h1')
        if h1_tag:
            h1_text = h1_tag.get_text().strip()
            if h1_text:
                title = h1_text

        result = {
            'url': url,
            'title': title,
            'content': text,
            'word_count': len(text.split())
        }

        logger.info(f"Extracted {result['word_count']} words from {url}")
        return result

    except Exception as e:
        logger.error(f"Error extracting text from {url}: {e}")
        return {
            'url': url,
            'title': 'Error',
            'content': '',
            'word_count': 0
        }


def chunk_text(text: str, chunk_size: int = 1000, overlap: int = 100) -> List[Dict[str, Any]]:
    """
    Split text into chunks of specified size with overlap.

    Args:
        text: The text to chunk
        chunk_size: Maximum size of each chunk (in characters)
        overlap: Number of characters to overlap between chunks

    Returns:
        List of text chunks with metadata
    """
    if not text:
        return []

    chunks = []
    start = 0

    while start < len(text):
        end = start + chunk_size

        # If this is the last chunk, include the rest
        if end >= len(text):
            end = len(text)
        else:
            # Try to break at sentence boundary first
            sentence_end = -1
            for punct in '.!?':
                last_punct = text.rfind(punct, start, end)
                if last_punct > sentence_end:
                    sentence_end = last_punct

            # If found a sentence boundary, use it plus a bit of space for context
            if sentence_end > start and sentence_end < end:
                end = sentence_end + 1
            else:
                # If no sentence boundary, try to break at word boundary
                temp_end = end
                while temp_end > start and text[temp_end] not in ' \n\t':
                    temp_end -= 1

                # If we couldn't find a good word break, use the original end
                if temp_end > start:
                    end = temp_end
                else:
                    end = temp_end  # Just break at the original position

        chunk_text = text[start:end].strip()

        if chunk_text:  # Only add non-empty chunks
            chunk = {
                'text': chunk_text,
                'start_pos': start,
                'end_pos': end,
                'length': len(chunk_text)
            }
            chunks.append(chunk)

        # Move start position forward by chunk_size minus overlap
        start = end - overlap if end < len(text) else len(text)

        # If we've made no progress, move forward by chunk_size to avoid infinite loop
        if start <= end - overlap and end >= len(text):
            break
        elif start == end - overlap and overlap == 0:
            start = end

    logger.info(f"Created {len(chunks)} chunks from text")
    return chunks


def embed(texts: List[str]) -> List[List[float]]:
    """
    Generate embeddings for a list of texts using Cohere.

    Args:
        texts: List of texts to embed

    Returns:
        List of embedding vectors
    """
    if not texts:
        return []

    logger.info(f"Generating embeddings for {len(texts)} texts")

    # Implement retry logic for Cohere API calls
    max_retries = 3
    for attempt in range(max_retries):
        try:
            response = co.embed(
                texts=texts,
                model="embed-english-v3.0",  # Using Cohere's English embedding model
                input_type="search_document"  # Appropriate for document search
            )

            embeddings = response.embeddings
            logger.info(f"Generated {len(embeddings)} embeddings successfully")

            # Validate embedding dimensions
            if embeddings and len(embeddings) > 0:
                expected_dim = 1024  # Cohere embeddings are 1024-dim for embed-english-v3.0
                for i, embedding in enumerate(embeddings):
                    if len(embedding) != expected_dim:
                        logger.warning(f"Embedding {i} has {len(embedding)} dimensions, expected {expected_dim}")

            return embeddings
        except Exception as e:
            logger.error(f"Attempt {attempt + 1} failed to generate embeddings: {e}")
            if attempt < max_retries - 1:
                time.sleep(2 ** attempt)  # Exponential backoff
                continue
            else:
                logger.error(f"All {max_retries} attempts failed to generate embeddings")
                return [[] for _ in range(len(texts))]  # Return empty embeddings in case of error


def create_collection(collection_name: str = "rag_embedding"):
    """
    Create a Qdrant collection for storing embeddings.

    Args:
        collection_name: Name of the collection to create
    """
    logger.info(f"Creating Qdrant collection: {collection_name}")

    # Implement retry logic for Qdrant operations
    max_retries = 3
    for attempt in range(max_retries):
        try:
            # Check if collection already exists
            collections = qdrant_client.get_collections()
            collection_exists = any(col.name == collection_name for col in collections.collections)

            if collection_exists:
                logger.info(f"Collection {collection_name} already exists, recreating...")
                qdrant_client.delete_collection(collection_name)

            # Create new collection with appropriate vector size (Cohere embeddings are 1024-dim)
            qdrant_client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),
            )

            logger.info(f"Collection {collection_name} created successfully")
            return  # Success, exit the function
        except Exception as e:
            logger.error(f"Attempt {attempt + 1} failed to create collection {collection_name}: {e}")
            if attempt < max_retries - 1:
                time.sleep(2 ** attempt)  # Exponential backoff
                continue
            else:
                logger.error(f"All {max_retries} attempts failed to create collection, trying fallback...")
                # Try with a simpler configuration as fallback
                try:
                    qdrant_client.recreate_collection(
                        collection_name=collection_name,
                        vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),
                    )
                    logger.info(f"Collection {collection_name} recreated with fallback method")
                    return  # Success with fallback
                except Exception as fallback_error:
                    logger.error(f"Fallback collection creation also failed: {fallback_error}")
                    raise


def save_chunk_to_qdrant(chunk_data: Dict[str, Any], embedding: List[float], collection_name: str = "rag_embedding"):
    """
    Save a chunk with its embedding to Qdrant.

    Args:
        chunk_data: Dictionary containing chunk information
        embedding: The embedding vector
        collection_name: Name of the collection to save to
    """
    # Implement retry logic for Qdrant operations
    max_retries = 3
    for attempt in range(max_retries):
        try:
            # Create a unique ID for this chunk
            chunk_id = str(uuid.uuid4())

            # Prepare the payload with metadata
            payload = {
                "url": chunk_data.get("url", ""),
                "title": chunk_data.get("title", ""),
                "content": chunk_data.get("text", "")[:500],  # Store first 500 chars as preview
                "start_pos": chunk_data.get("start_pos", 0),
                "end_pos": chunk_data.get("end_pos", 0),
                "chunk_length": chunk_data.get("length", 0),
                "created_at": time.time()
            }

            # Upsert the point to Qdrant
            qdrant_client.upsert(
                collection_name=collection_name,
                points=[
                    models.PointStruct(
                        id=chunk_id,
                        vector=embedding,
                        payload=payload
                    )
                ]
            )

            logger.debug(f"Saved chunk {chunk_id} to Qdrant")
            return chunk_id
        except Exception as e:
            logger.error(f"Attempt {attempt + 1} failed to save chunk to Qdrant: {e}")
            if attempt < max_retries - 1:
                time.sleep(2 ** attempt)  # Exponential backoff
                continue
            else:
                logger.error(f"All {max_retries} attempts failed to save chunk to Qdrant")
                return None


def main():
    """
    Main execution function that orchestrates the entire pipeline.
    """
    logger.info("Starting embedding pipeline")

    # Configuration
    target_url = os.getenv("TARGET_URL", "https://physical-ai-ivory.vercel.app/")
    collection_name = "rag_embedding"
    chunk_size = int(os.getenv("CHUNK_SIZE", "1000"))
    chunk_overlap = int(os.getenv("CHUNK_OVERLAP", "100"))

    # Step 1: Create Qdrant collection
    create_collection(collection_name)

    # Step 2: Discover URLs
    urls = get_all_urls(target_url)

    # Step 3: Process each URL
    processed_count = 0
    for i, url in enumerate(urls):
        logger.info(f"Processing URL {i+1}/{len(urls)}: {url}")

        # Extract text from URL with retry
        max_retries = 2
        page_data = None
        for attempt in range(max_retries):
            try:
                page_data = extract_text_from_url(url)
                break  # Success, exit retry loop
            except Exception as e:
                logger.warning(f"Attempt {attempt + 1} failed to extract text from {url}: {e}")
                if attempt < max_retries - 1:
                    time.sleep(2 ** attempt)  # Exponential backoff
                    continue
                else:
                    logger.error(f"All {max_retries} attempts failed for {url}, skipping")
                    page_data = {
                        'url': url,
                        'title': 'Error',
                        'content': '',
                        'word_count': 0
                    }

        if not page_data['content']:
            logger.warning(f"No content found for {url}, skipping")
            continue

        # Chunk the text using configuration values
        chunks = chunk_text(page_data['content'], chunk_size=chunk_size, overlap=chunk_overlap)

        # Process each chunk with memory management
        for j, chunk in enumerate(chunks):
            # Add page metadata to chunk
            chunk_with_meta = {
                **chunk,
                'url': page_data['url'],
                'title': page_data['title']
            }

            # Generate embedding
            embedding = embed([chunk['text']])[0]  # Get first (and only) embedding

            if embedding and len(embedding) > 0:
                # Save to Qdrant
                chunk_id = save_chunk_to_qdrant(chunk_with_meta, embedding, collection_name)
                if chunk_id:
                    logger.debug(f"Successfully saved chunk {j+1} from {url}")

                # Clear variables to manage memory
                del embedding
                del chunk_id
            else:
                logger.warning(f"Failed to generate embedding for chunk {j+1} from {url}")

        # Clear large variables to manage memory
        del page_data
        del chunks

        processed_count += 1

        # Add a small delay to be respectful to the server
        time.sleep(0.1)

    logger.info(f"Pipeline completed! Processed {processed_count}/{len(urls)} URLs successfully")


if __name__ == "__main__":
    main()