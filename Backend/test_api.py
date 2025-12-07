import requests
import json

def test_backend_connection():
    """Test the connection to the backend API"""
    print("Testing backend API connection...")

    # Test health endpoint
    try:
        response = requests.get("http://localhost:8000/health")
        if response.status_code == 200:
            print("SUCCESS: Backend API is reachable")
            print(f"Health check response: {response.json()}")
        else:
            print(f"ERROR: Backend API returned status code: {response.status_code}")
    except requests.exceptions.ConnectionError:
        print("ERROR: Cannot connect to backend API. Is the server running on http://localhost:8000?")
        print("To start the backend server, run: python -m uvicorn main:app --host 0.0.0.0 --port 8000")

    # Test query endpoint structure
    print("\nQuery endpoint structure (for frontend integration):")
    print("POST http://localhost:8000/query")
    print("Headers: {'Content-Type': 'application/json'}")
    print("Body: {")
    print("  'query': 'your question here',")
    print("  'session_id': 'unique session id',")
    print("  'selected_text': 'optional selected text',")
    print("  'top_k': 10,")
    print("  'temperature': 0.1")
    print("}")

if __name__ == "__main__":
    test_backend_connection()