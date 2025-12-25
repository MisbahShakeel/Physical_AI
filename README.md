# Physical AI and Robotics Learning Framework

A comprehensive educational platform that combines a Docusaurus-based frontend documentation site with a FastAPI-based backend RAG (Retrieval-Augmented Generation) system. The platform provides interactive learning experiences for students studying Physical AI and Robotics.

## 🚀 Features

- **Interactive Chatbot**: AI-powered assistant to answer questions about robotics concepts
- **4 Comprehensive Modules**:
  - Module 1: ROS 2 Foundations and Basic Robot Control
  - Module 2: Robot Simulation and Perception
  - Module 3: Embodied AI and Human-Robot Interaction
  - Module 4: Advanced AI Integration and Capstone Project
- **Citation System**: Responses include references to specific textbook sections
- **Session Management**: Maintains conversation history

## 🏗️ Architecture

### Frontend (Docusaurus)
- **Technology Stack**: React, TypeScript, CSS Modules
- **Purpose**: Documentation site with interactive learning materials
- **Key Components**:
  - Textbook content organized in modules and chapters
  - Interactive chat widget for Q&A
  - Navigation system for course materials

### Backend (FastAPI RAG System)
- **Technology Stack**: FastAPI, PostgreSQL, Qdrant, OpenAI API
- **Purpose**: Knowledge base and AI-powered question answering
- **Key Components**:
  - Document ingestion system
  - Vector database for retrieval
  - AI-powered response generation
  - Session management

## 🛠️ Development Setup

### Prerequisites
- Node.js (v16 or higher)
- Python (v3.8 or higher)
- Git
- Docker (for Qdrant vector database)

### Backend Setup
1. Navigate to the backend directory:
   ```bash
   cd backend/app
   ```

2. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Set up environment variables in a `.env` file (see `.env.example` for reference):
   ```env
   COHERE_API_KEY=your_cohere_api_key
   GEMINI_API_KEY=your_gemini_api_key
   QDRANT_URL=http://localhost:6333  # For local development
   QDRANT_API_KEY=your_qdrant_api_key_if_using_auth
   ```

4. Start Qdrant vector database:
   ```bash
   docker-compose up -d
   ```

   If Docker is not available, you can use a remote Qdrant instance by updating the QDRANT_URL in your .env file.

5. Populate the database with documents (run once to index the textbook content):
   ```bash
   python main.py
   ```

6. Start the backend server:
   ```bash
   python -m uvicorn api:app --host 0.0.0.0 --port 8000
   ```

### Frontend Setup
1. Navigate to the frontend directory:
   ```bash
   cd Frontend/Physical_AI
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Start the development server:
   ```bash
   npm start
   ```

## 🌐 Vercel Deployment

### Frontend-only Deployment (with mock API)
1. The chat widget includes simulated responses for Vercel deployment without a backend
2. Simply connect your GitHub repository to Vercel
3. The site will deploy automatically

### Frontend + Backend Deployment
1. Deploy the backend to a service like Render, Railway, or AWS
2. Deploy the frontend to Vercel
3. Set the `REACT_APP_API_BASE_URL` environment variable in Vercel to point to your backend

## 📚 Modules Overview

### Module 1: ROS 2 Foundations and Basic Robot Control
- Introduction to ROS 2 concepts
- Node communication (topics, services, actions)
- Robot control fundamentals
- Basic navigation and manipulation

### Module 2: Robot Simulation and Perception
- Gazebo simulation environments
- Sensor integration and data processing
- Perception algorithms
- Environment modeling

### Module 3: Embodied AI and Human-Robot Interaction
- NVIDIA Isaac platform integration
- AI decision making for robotics
- Human-robot interaction principles
- Natural language processing for robotics

### Module 4: Advanced AI Integration and Capstone Project
- Vision-Language-Action (VLA) systems
- Advanced perception and planning
- Capstone project implementation
- Integration of all learned concepts

## 🤖 Chat Widget Features

- **Contextual Responses**: Answers based on textbook content
- **Citations**: References to specific chapters and sections
- **Session Management**: Maintains conversation history
- **Responsive Design**: Works on all device sizes

## 🔧 Configuration

### Environment Variables
- `REACT_APP_API_BASE_URL`: Production backend API URL (optional, for Vercel deployment)

### API Endpoints
- `POST /query`: Main endpoint for textbook Q&A
- `GET /health`: System health monitoring
- `POST /ingest`: Document ingestion into knowledge base
- `POST /embed`: Text embedding generation
- `POST /feedback`: User feedback collection

## 📈 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Commit your changes (`git commit -m 'Add some amazing feature'`)
5. Push to the branch (`git push origin feature/amazing-feature`)
6. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 📞 Support

For support, please open an issue in the GitHub repository.

---

Built with ❤️ using Docusaurus, FastAPI, and the latest in AI technology.