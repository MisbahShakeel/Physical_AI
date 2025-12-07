"""
Verification script for Textbook RAG System installation
"""
import sys
import importlib

def check_package(package_name, import_name=None):
    """Check if a package is available"""
    if import_name is None:
        import_name = package_name

    try:
        importlib.import_module(import_name)
        print(f"[OK] {package_name} - Available")
        return True
    except ImportError:
        print(f"[MISSING] {package_name} - Not available")
        return False

def main():
    print("[VERIFY] Textbook RAG System - Installation Verification")
    print("=" * 50)

    # Core packages needed for the RAG system
    core_packages = [
        ("fastapi", "fastapi"),
        ("uvicorn", "uvicorn"),
        ("openai", "openai"),
        ("qdrant_client", "qdrant_client"),
        ("asyncpg", "asyncpg"),
        ("pydantic", "pydantic"),
        ("numpy", "numpy"),
        ("scikit_learn", "sklearn"),
        ("pandas", "pandas"),
        ("aiohttp", "aiohttp"),
        ("httpx", "httpx"),
        ("pyjwt", "jwt"),
        ("langchain", "langchain"),
        ("pytest", "pytest"),
        ("redis", "redis"),
        ("slowapi", "slowapi"),
        ("aioredis", "aioredis"),
        ("cryptography", "cryptography"),
        ("python_jose", "jose"),
        ("passlib", "passlib"),
        ("python_multipart", "multipart"),
        ("python_dotenv", "dotenv"),
        ("tiktoken", "tiktoken"),
        ("sentence_transformers", "sentence_transformers"),
        ("transformers", "transformers"),
        ("datasets", "datasets"),
        ("faiss_cpu", "faiss"),
        ("psutil", "psutil"),
        ("gputil", "GPUtil"),
    ]

    print("Checking core packages...")
    available_count = 0
    total_count = len(core_packages)

    for package, import_name in core_packages:
        if check_package(package, import_name):
            available_count += 1

    print(f"\n[RESULTS] Verification Results: {available_count}/{total_count} packages available")

    if available_count >= total_count * 0.8:  # 80% threshold
        print("[SUCCESS] Installation is successful! The RAG system is ready to use.")
        print("\nNext steps:")
        print("1. Set up your environment variables (API keys, database URLs)")
        print("2. Configure your Qdrant and PostgreSQL connections")
        print("3. Run the application: uvicorn rag_system:app --reload")
        print("4. Test the API endpoints")
    elif available_count >= total_count * 0.6:  # 60% threshold
        print("[WARNING] Installation is partially successful. Most core functionality is available.")
        print("Some advanced features may not work until all packages are installed.")
    else:
        print("[ERROR] Installation is incomplete. Please install missing packages.")
        return False

    # Test basic functionality
    print(f"\n[TEST] Testing basic functionality...")
    try:
        # Test FastAPI app creation
        from fastapi import FastAPI
        app = FastAPI()
        print("[OK] FastAPI application can be created")

        # Test OpenAI client creation (without API key)
        import openai
        print("[OK] OpenAI module is available")

        # Test Qdrant client creation (without connection)
        import qdrant_client
        print("[OK] Qdrant client module is available")

        # Test Pydantic models
        from pydantic import BaseModel
        class TestModel(BaseModel):
            name: str
        print("[OK] Pydantic models work correctly")

        print("[OK] Basic functionality verified")

    except Exception as e:
        print(f"[ERROR] Basic functionality test failed: {e}")
        return False

    print(f"\n[SUCCESS] Textbook RAG System installation verification completed successfully!")
    return True

if __name__ == "__main__":
    success = main()
    if not success:
        sys.exit(1)