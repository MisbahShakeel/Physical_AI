"""
OpenAI Agent Orchestration System for Textbook RAG
"""
from openai import OpenAI
from typing import Dict, Any, List, Optional
import json
import asyncio
from datetime import datetime


class AgentOrchestrator:
    def __init__(self, openai_client: OpenAI, retriever, postgres_client):
        self.openai = openai_client
        self.retriever = retriever
        self.postgres = postgres_client

        # Initialize specialized agents
        self.query_router = QueryRoutingAgent(openai_client)
        self.retrieval_agent = RetrievalAgent(retriever)
        self.synthesis_agent = AnswerSynthesisAgent(openai_client)
        self.verification_agent = VerificationAgent(openai_client)

    async def process_query(
        self,
        query: str,
        session_id: str,
        selected_text: Optional[str] = None
    ) -> Dict[str, Any]:
        """Main orchestration method"""

        # 1. Route query to determine processing path
        routing_result = await self.query_router.route_query(query, selected_text)

        # 2. Retrieve relevant context
        retrieval_result = await self.retrieval_agent.retrieve(
            query=query,
            search_mode=routing_result['search_mode'],
            selected_text=selected_text
        )

        # 3. Synthesize answer with grounding
        synthesis_result = await self.synthesis_agent.generate_answer(
            query=query,
            context_chunks=retrieval_result['chunks'],
            routing_info=routing_result
        )

        # 4. Verify hallucinations and citations
        verification_result = await self.verification_agent.verify_response(
            query=query,
            response=synthesis_result['response'],
            context_chunks=retrieval_result['chunks']
        )

        # 5. Log query and return response
        await self.postgres.log_query({
            'session_id': session_id,
            'query_text': query,
            'response_text': verification_result['final_response'],
            'citations': verification_result['citations'],
            'query_type': routing_result['search_mode']
        })

        return {
            'response': verification_result['final_response'],
            'citations': verification_result['citations'],
            'search_mode': routing_result['search_mode'],
            'retrieval_metadata': retrieval_result['metadata']
        }


class QueryRoutingAgent:
    def __init__(self, openai_client: OpenAI):
        self.openai = openai_client

    async def route_query(self, query: str, selected_text: Optional[str]) -> Dict[str, Any]:
        """Determine query processing path"""

        # If selected text is provided, always use selected-text mode
        if selected_text and len(selected_text.strip()) > 10:
            return {
                'search_mode': 'selected_text',
                'selected_text': selected_text
            }

        # Otherwise, determine if it's a general knowledge query
        routing_prompt = f"""
        Analyze the following query and determine the appropriate search mode:

        Query: {query}

        Available modes:
        - 'global': Search the entire textbook for general knowledge
        - 'selected_text': Search only within user-selected text

        If the user has provided specific text context, always return 'selected_text'.
        If no specific context is provided, return 'global'.

        Respond with a JSON object containing 'search_mode' field.
        """

        response = self.openai.chat.completions.create(
            model="gpt-4o",
            messages=[{"role": "user", "content": routing_prompt}],
            response_format={"type": "json_object"}
        )

        result = json.loads(response.choices[0].message.content)
        result['search_mode'] = 'global'  # Default to global if not specified

        return result


class RetrievalAgent:
    def __init__(self, retriever):
        self.retriever = retriever

    async def retrieve(
        self,
        query: str,
        search_mode: str,
        selected_text: Optional[str] = None
    ) -> Dict[str, Any]:
        """Perform retrieval based on search mode"""

        if search_mode == 'selected_text' and selected_text:
            chunks = await self.retriever.retrieve_from_selected_text(
                query=query,
                selected_text=selected_text,
                top_k=5
            )
        else:
            chunks = await self.retriever.semantic_search(
                query=query,
                top_k=10,
                selected_text_filter=selected_text
            )

        return {
            'chunks': chunks,
            'metadata': {
                'search_mode': search_mode,
                'retrieved_count': len(chunks)
            }
        }


class AnswerSynthesisAgent:
    def __init__(self, openai_client: OpenAI):
        self.openai = openai_client

    async def generate_answer(
        self,
        query: str,
        context_chunks: List[Dict[str, Any]],
        routing_info: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Generate grounded answer from context"""

        # Format context with citations
        formatted_context = self.format_context_with_citations(context_chunks)

        system_prompt = """
        You are an expert assistant for a technical textbook. Your role is to provide accurate,
        well-cited answers based strictly on the provided context.

        RULES:
        1. Only use information from the provided context chunks
        2. Never hallucinate or invent information
        3. Always cite specific chunks when referencing information
        4. If the answer cannot be found in the context, clearly state this
        5. Maintain academic tone appropriate for technical content
        6. Structure responses clearly with relevant sections
        """

        user_prompt = f"""
        Query: {query}

        Context Chunks:
        {formatted_context}

        Provide a comprehensive answer that:
        1. Directly addresses the query
        2. Uses information only from the provided context
        3. Includes specific citations for each piece of information
        4. Maintains the academic tone of the textbook
        """

        response = self.openai.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ]
        )

        return {
            'response': response.choices[0].message.content,
            'context_chunks': context_chunks
        }

    def format_context_with_citations(self, chunks: List[Dict[str, Any]]) -> str:
        """Format context chunks with proper citations"""
        formatted = []
        for i, chunk in enumerate(chunks):
            citation = f"[Chunk {i+1}: {chunk['chapter_title']} - {chunk['section_title']}]"
            formatted.append(f"{citation}\n{chunk['content']}\n")

        return "\n".join(formatted)


class VerificationAgent:
    def __init__(self, openai_client: OpenAI):
        self.openai = openai_client

    async def verify_response(
        self,
        query: str,
        response: str,
        context_chunks: List[Dict[str, Any]]
    ) -> Dict[str, Any]:
        """Verify response for hallucinations and proper citations"""

        verification_prompt = f"""
        Verify the following response for hallucinations and citation accuracy:

        Query: {query}

        Response: {response}

        Context Chunks:
        {self.format_chunks_for_verification(context_chunks)}

        Check for:
        1. Information in response that's not supported by context chunks
        2. Citations that don't match the provided chunks
        3. Factual accuracy based on context
        4. Proper attribution of information

        Provide verification result in JSON format with:
        - 'is_valid': boolean indicating if response is valid
        - 'issues': array of identified issues
        - 'citations': array of properly cited chunk references
        - 'final_response': corrected response if needed
        """

        verification_response = self.openai.chat.completions.create(
            model="gpt-4o",
            messages=[{"role": "user", "content": verification_prompt}],
            response_format={"type": "json_object"}
        )

        verification_result = json.loads(verification_response.choices[0].message.content)

        return {
            'final_response': verification_result.get('final_response', response),
            'citations': verification_result.get('citations', []),
            'is_valid': verification_result.get('is_valid', True),
            'issues': verification_result.get('issues', [])
        }

    def format_chunks_for_verification(self, chunks: List[Dict[str, Any]]) -> str:
        """Format chunks for verification agent"""
        return "\n".join([
            f"Chunk {i+1}: {chunk['content'][:200]}..."
            for i, chunk in enumerate(chunks)
        ])


class ContextualAgent:
    """Handles contextual understanding and session management"""

    def __init__(self, postgres_client):
        self.postgres = postgres_client

    async def get_conversation_context(self, session_id: str, max_turns: int = 5) -> List[Dict[str, str]]:
        """Retrieve recent conversation history for context"""
        # This would fetch recent queries/responses from the database
        # For now, returning empty list
        return []

    async def update_conversation_context(self, session_id: str, query: str, response: str):
        """Update conversation context in session"""
        # This would store the query-response pair in session context
        pass


class KnowledgeGraphAgent:
    """Manages relationships between concepts in the textbook"""

    def __init__(self, openai_client: OpenAI, postgres_client):
        self.openai = openai_client
        self.postgres = postgres_client

    async def extract_concepts(self, text: str) -> List[str]:
        """Extract key concepts from text"""
        prompt = f"""
        Extract the key technical concepts from the following text:

        Text: {text[:1000]}

        Return a JSON array of key concepts mentioned in the text.
        """

        response = self.openai.chat.completions.create(
            model="gpt-4o",
            messages=[{"role": "user", "content": prompt}],
            response_format={"type": "json_object"}
        )

        result = json.loads(response.choices[0].message.content)
        return result.get('concepts', [])

    async def build_knowledge_graph(self, chunks: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Build a knowledge graph from chunks"""
        concepts = []
        for chunk in chunks:
            chunk_concepts = await self.extract_concepts(chunk['content'])
            concepts.extend(chunk_concepts)

        # Remove duplicates while preserving order
        unique_concepts = list(dict.fromkeys(concepts))

        # Build relationships between concepts
        relationships = await self.identify_relationships(unique_concepts)

        return {
            'concepts': unique_concepts,
            'relationships': relationships
        }

    async def identify_relationships(self, concepts: List[str]) -> List[Dict[str, Any]]:
        """Identify relationships between concepts"""
        if len(concepts) < 2:
            return []

        # For simplicity, we'll create a basic relationship matrix
        # In a real implementation, this would use more sophisticated NLP
        relationships = []
        for i, concept1 in enumerate(concepts):
            for j, concept2 in enumerate(concepts[i+1:], i+1):
                relationship = {
                    'concept1': concept1,
                    'concept2': concept2,
                    'relationship_type': 'related_to',
                    'strength': 0.5  # Default strength
                }
                relationships.append(relationship)

        return relationships


class MultiModalAgent:
    """Handles multi-modal inputs (text, images, code)"""

    def __init__(self, openai_client: OpenAI):
        self.openai = openai_client

    async def process_multimodal_input(self, query: str, media_content: Optional[str] = None) -> str:
        """Process queries that may include images, code, or other media"""
        if media_content:
            # Process multimodal query
            prompt = f"""
            Process the following query with associated media content:

            Query: {query}
            Media Content: {media_content}

            Provide a response that addresses both the text query and the media content.
            """

            response = self.openai.chat.completions.create(
                model="gpt-4o",
                messages=[{"role": "user", "content": prompt}]
            )

            return response.choices[0].message.content
        else:
            # Just return the original query for standard processing
            return query


class FeedbackLearningAgent:
    """Learns from user feedback to improve responses"""

    def __init__(self, postgres_client):
        self.postgres = postgres_client

    async def process_feedback(self, query_id: str, rating: int, comment: Optional[str] = None):
        """Process user feedback to improve the system"""
        # Store feedback in database
        await self.postgres.log_feedback({
            'query_id': query_id,
            'rating': rating,
            'comment': comment
        })

        # If negative feedback, analyze why the response was poor
        if rating == -1 and comment:
            await self.analyze_negative_feedback(query_id, comment)

    async def analyze_negative_feedback(self, query_id: str, comment: str):
        """Analyze negative feedback to identify improvement areas"""
        # This would analyze the feedback and potentially update system parameters
        # For now, just log the analysis
        pass

    async def get_improvement_suggestions(self) -> List[Dict[str, Any]]:
        """Get suggestions for system improvements based on feedback"""
        # Analyze feedback patterns to suggest improvements
        feedback_analysis = await self.postgres.analyze_feedback()

        suggestions = []
        if feedback_analysis.get('common_issues'):
            for issue in feedback_analysis['common_issues']:
                suggestions.append({
                    'issue': issue,
                    'suggested_fix': f"Improve handling of {issue}",
                    'priority': 'high' if issue in ['hallucination', 'irrelevant'] else 'medium'
                })

        return suggestions


class QualityAssuranceAgent:
    """Ensures response quality and consistency"""

    def __init__(self, openai_client: OpenAI):
        self.openai = openai_client

    async def assess_response_quality(self, query: str, response: str, context: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Assess the quality of a response"""
        quality_prompt = f"""
        Assess the quality of the following response to the query based on the provided context.

        Query: {query}

        Response: {response}

        Context Chunks:
        {self.format_context_for_qa(context)}

        Evaluate the response on these dimensions:
        1. Accuracy - How factually correct is the response?
        2. Relevance - How relevant is the response to the query?
        3. Completeness - How comprehensive is the response?
        4. Clarity - How clear and well-structured is the response?
        5. Citation - Are sources properly cited?

        Provide scores from 1-5 for each dimension and an overall quality score.
        Return the assessment in JSON format.
        """

        assessment = self.openai.chat.completions.create(
            model="gpt-4o",
            messages=[{"role": "user", "content": quality_prompt}],
            response_format={"type": "json_object"}
        )

        return json.loads(assessment.choices[0].message.content)

    def format_context_for_qa(self, context: List[Dict[str, Any]]) -> str:
        """Format context for quality assessment"""
        return "\n".join([
            f"Chunk {i+1}: {chunk['content'][:200]}..."
            for i, chunk in enumerate(context)
        ])


# Agent Coordinator - manages all agents
class AgentCoordinator:
    def __init__(self, openai_client: OpenAI, retriever, postgres_client):
        self.openai = openai_client
        self.retriever = retriever
        self.postgres = postgres_client

        # Initialize all specialized agents
        self.orchestrator = AgentOrchestrator(openai_client, retriever, postgres_client)
        self.contextual_agent = ContextualAgent(postgres_client)
        self.knowledge_graph_agent = KnowledgeGraphAgent(openai_client, postgres_client)
        self.multimodal_agent = MultiModalAgent(openai_client)
        self.feedback_agent = FeedbackLearningAgent(postgres_client)
        self.qa_agent = QualityAssuranceAgent(openai_client)

    async def process_complex_query(
        self,
        query: str,
        session_id: str,
        selected_text: Optional[str] = None,
        media_content: Optional[str] = None
    ) -> Dict[str, Any]:
        """Process a complex query using multiple agents"""

        # Process multimodal input if present
        processed_query = await self.multimodal_agent.process_multimodal_input(query, media_content)

        # Get conversation context
        conversation_context = await self.contextual_agent.get_conversation_context(session_id)

        # Process through main orchestrator
        result = await self.orchestrator.process_query(
            query=processed_query,
            session_id=session_id,
            selected_text=selected_text
        )

        # Build knowledge graph from retrieved chunks
        knowledge_graph = await self.knowledge_graph_agent.build_knowledge_graph(result['citations'])
        result['knowledge_graph'] = knowledge_graph

        # Assess response quality
        quality_assessment = await self.qa_agent.assess_response_quality(
            query=processed_query,
            response=result['response'],
            context=result['citations']
        )
        result['quality_assessment'] = quality_assessment

        # Update conversation context
        await self.contextual_agent.update_conversation_context(
            session_id, processed_query, result['response']
        )

        return result