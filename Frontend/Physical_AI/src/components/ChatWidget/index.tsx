import React, { useState, useEffect, useRef } from 'react';
import styles from './styles.module.css';

interface Message {
  id: string;
  content: string;
  sender: 'user' | 'assistant';
  timestamp: Date;
  citations?: Array<{
    chapter_title: string;
    section_title: string;
    page_number?: number;
  }>;
}

// Function to simulate API call for Vercel deployment without backend
const simulateApiCall = async (query: string, selectedText?: string): Promise<any> => {
  // Simulate network delay
  await new Promise(resolve => setTimeout(resolve, 500 + Math.random() * 1000));

  const modules = [
    "Module 1: ROS 2 Fundamentals",
    "Module 2: Simulation Environments",
    "Module 3: NVIDIA Isaac Integration",
    "Module 4: Vision-Language-Action Systems"
  ];

  const chapters = ["Introduction", "Core Concepts", "Implementation", "Advanced Topics"];

  // Generate mock response based on query content
  let responseText = "Based on the textbook content, ";

  if (query.toLowerCase().includes('ros')) {
    responseText += "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It provides a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms. Key concepts include nodes, topics, services, and actions for inter-process communication.";
  } else if (query.toLowerCase().includes('simulation') || query.toLowerCase().includes('gazebo')) {
    responseText += "Gazebo is a 3D simulation environment that enables accurate and efficient simulation of robots and environments. It provides physics simulation, sensor simulation, and realistic rendering capabilities essential for robotics development. Gazebo helps in testing algorithms before deploying to real robots, reducing development time and costs.";
  } else if (query.toLowerCase().includes('nvidia') || query.toLowerCase().includes('isaac')) {
    responseText += "NVIDIA Isaac is a robotics platform that accelerates AI-powered robotics development. It includes Isaac Sim for photorealistic simulation, Isaac ROS for hardware-accelerated perception, and Isaac Lab for robot learning applications. The platform leverages GPU acceleration for computationally intensive tasks like computer vision and deep learning.";
  } else if (query.toLowerCase().includes('vision') || query.toLowerCase().includes('language') || query.toLowerCase().includes('action')) {
    responseText += "Vision-Language-Action (VLA) systems represent an emerging paradigm in robotics where visual input, natural language understanding, and action execution are tightly integrated to enable more intuitive human-robot interaction. These systems allow robots to understand complex commands expressed in natural language and act appropriately in dynamic environments.";
  } else {
    responseText += "the Physical AI and Robotics framework covers advanced topics in artificial intelligence applied to robotics. This includes robot perception, planning, control, and learning systems that enable autonomous behavior in complex environments.";
  }

  // Generate mock citations
  const citations = Array.from({ length: 3 }, () => ({
    chapter_title: chapters[Math.floor(Math.random() * chapters.length)],
    section_title: `Section ${Math.floor(Math.random() * 5) + 1}`,
    page_number: Math.floor(Math.random() * 190) + 10,
    module: modules[Math.floor(Math.random() * modules.length)]
  }));

  return {
    response: responseText,
    citations,
    search_mode: selectedText ? 'selected_text' : 'global',
    latency_ms: Math.floor(Math.random() * 100) + 50, // Random latency between 50-150ms
    query_id: `query-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`
  };
};

const ChatWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState<string>('');
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  // Initialize session
  useEffect(() => {
    // Generate or retrieve session ID
    const storedSessionId = localStorage.getItem('chat-session-id');
    if (storedSessionId) {
      setSessionId(storedSessionId);
    } else {
      const newSessionId = `session-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
      localStorage.setItem('chat-session-id', newSessionId);
      setSessionId(newSessionId);
    }
  }, []);

  // Scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    setInputValue(e.target.value);
  };

  const toggleButton = () => {
    setIsOpen(!isOpen);
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message
    const userMessage: Message = {
      id: `msg-${Date.now()}`,
      content: inputValue,
      sender: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Get selected text if any
      const selectedText = window.getSelection()?.toString().trim();

      // Determine if we should use the backend API or mock responses
      // Check if we have a production API URL configured
      const hasProductionApi = typeof window !== 'undefined' && process.env.REACT_APP_API_BASE_URL;

      let data;
      if (hasProductionApi) {
        // Use production backend API
        const apiBaseUrl = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000';
        const response = await fetch(`${apiBaseUrl}/query`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            query: inputValue,
            session_id: sessionId,
            selected_text: selectedText || null,
            top_k: 10,
            temperature: 0.1,
          }),
        });

        if (!response.ok) {
          throw new Error(`API error: ${response.status}`);
        }

        data = await response.json();
      } else {
        // Use mock responses for Vercel deployment without backend
        data = await simulateApiCall(inputValue, selectedText);
      }

      // Add assistant message
      const assistantMessage: Message = {
        id: `msg-${Date.now() + 1}`,
        content: data.response,
        sender: 'assistant',
        timestamp: new Date(),
        citations: data.citations || [],
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Chat error:', error);

      const errorMessage: Message = {
        id: `msg-${Date.now() + 1}`,
        content: 'Sorry, I encountered an error. Please try again.',
        sender: 'assistant',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const formatCitations = (citations: Message['citations']) => {
    if (!citations || citations.length === 0) return null;

    return (
      <div className={styles.citations}>
        <small>Sources: {citations.map(c =>
          `${c.chapter_title} - ${c.section_title}`
        ).join(', ')}</small>
      </div>
    );
  };

  return (
    <div className={styles.chatWidget}>
      {isOpen ? (
        <div className={styles.chatContainer}>
          <div className={styles.chatHeader}>
            <h3>Physical AI Assistant</h3>
            <button onClick={toggleChat} className={styles.closeButton}>
              ×
            </button>
          </div>

          <div className={styles.chatMessages}>
            {messages.length === 0 ? (
              <div className={styles.welcomeMessage}>
                <p>Hello! I'm your Physical AI and Robotics Learning Assistant.</p>
                <p>Ask me anything about the textbook content:</p>
                <ul>
                  <li>ROS 2 fundamentals</li>
                  <li>Simulation environments</li>
                  <li>NVIDIA Isaac integration</li>
                  <li>Vision-Language-Action systems</li>
                </ul>
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  className={`${styles.message} ${styles[`${message.sender}Message`]}`}
                >
                  <div className={styles.content}>
                    {message.content}
                  </div>
                  {message.sender === 'assistant' && message.citations && formatCitations(message.citations)}
                  <div className={styles.timestamp}>
                    {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                  </div>
                </div>
              ))
            )}
            {isLoading && (
              <div className={`${styles.message} ${styles.assistantMessage}`}>
                <div className={styles.typingIndicator}>
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <form onSubmit={handleSubmit} className={styles.chatForm}>
            <input
              type="text"
              value={inputValue}
              onChange={handleInputChange}
              placeholder="Ask about the textbook content..."
              disabled={isLoading}
              className={styles.chatInput}
            />
            <button
              type="submit"
              disabled={isLoading || !inputValue.trim()}
              className={styles.sendButton}
            >
              Send
            </button>
          </form>
        </div>
      ) : (
        <button onClick={toggleChat} className={styles.toggleButton}>
          💬
        </button>
      )}
    </div>
  );
};

export default ChatWidget;