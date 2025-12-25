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
    module?: string;
    url?: string;
    content_preview?: string;
    score?: number;
  }>;
}

const ChatWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState<string>('');
  const [selectedText, setSelectedText] = useState<string>('');
  const [showSelectionButton, setShowSelectionButton] = useState(false);
  const [selectionPosition, setSelectionPosition] = useState({ x: 0, y: 0 });
  const messagesEndRef = useRef<HTMLDivElement>(null);
  // Hardcoded English text instead of translation
  const t = (key: string) => {
    const translations: Record<string, string> = {
      'Connection error': 'Connection error',
      'Failed to connect to backend API': 'Failed to connect to backend API',
      "I'm having trouble processing your request. Please try again.": "I'm having trouble processing your request. Please try again.",
      'Sources': 'Sources',
      'Unknown': 'Unknown',
      'Physical AI Assistant': 'Physical AI Assistant',
      'Close': 'Close',
      "Hello! I'm your Physical AI and Robotics Learning Assistant.": "Hello! I'm your Physical AI and Robotics Learning Assistant.",
      "Ask me anything about the textbook content:": "Ask me anything about the textbook content:",
      'ROS 2 fundamentals': 'ROS 2 fundamentals',
      'Simulation environments': 'Simulation environments',
      'NVIDIA Isaac integration': 'NVIDIA Isaac integration',
      'Vision-Language-Action systems': 'Vision-Language-Action systems',
      "Ask about the textbook content...": "Ask about the textbook content...",
      'Send': 'Send',
    };
    return translations[key] || key;
  };
  const currentLanguage = 'en'; // Default to English

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  // Initialize session
  useEffect(() => {
    const storedSessionId = localStorage.getItem('chat-session-id');
    if (storedSessionId) {
      setSessionId(storedSessionId);
    } else {
      const newSessionId = `session-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
      localStorage.setItem('chat-session-id', newSessionId);
      setSessionId(newSessionId);
    }
  }, []);

  // Handle text selection
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      if (selection && selection.toString().trim() !== '') {
        const selectedText = selection.toString().trim();
        if (selectedText.length > 0) {
          const range = selection.getRangeAt(0);
          const rect = range.getBoundingClientRect();
          setSelectionPosition({ x: rect.right, y: rect.top });
          setSelectedText(selectedText);
          setShowSelectionButton(true);
        } else {
          setShowSelectionButton(false);
        }
      } else {
        setShowSelectionButton(false);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
    };
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

  const handleSelectionQuery = () => {
    if (selectedText) {
      // Add the selected text as a user message
      const userMessage: Message = {
        id: `msg-${Date.now()}`,
        content: selectedText,
        sender: 'user',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, userMessage]);
      setIsOpen(true); // Open chat if it's closed
      setSelectedText('');
      setShowSelectionButton(false);

      // Set the selected text as input value so user can modify it before sending
      setInputValue(selectedText);

      // Focus the input field to allow immediate editing
      setTimeout(() => {
        const inputElement = document.querySelector(`.${styles.chatInput}`) as HTMLInputElement;
        if (inputElement) {
          inputElement.focus();
          // Move cursor to end of text
          inputElement.setSelectionRange(inputElement.value.length, inputElement.value.length);
        }
      }, 100);
    }
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

    setInputValue('');
    setIsLoading(true);

    try {
      // Connect to backend API - default to port 8000 where backend runs
      const apiBaseUrl = typeof window !== 'undefined'
                        ? window.location.protocol + '//' + window.location.hostname + ':8000'
                        : 'http://localhost:8000';
      let data;

      try {
        console.log('Sending request to backend:', `${apiBaseUrl}/query`, { query: inputValue, top_k: 10 });

        // Prepare context from previous messages for better conversation flow
        const contextMessages = messages.slice(-5); // Use last 5 messages as context
        const context = contextMessages.map(msg => `${msg.sender}: ${msg.content}`).join('\n');

        const requestBody: any = {
          query: inputValue,
          top_k: 10,
          language: currentLanguage // Include current language in request
        };

        // Include conversation context if available
        if (context.trim()) {
          requestBody.context = context;
        }

        const response = await fetch(`${apiBaseUrl}/query`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify(requestBody)
        });

        console.log('Response status:', response.status);

        if (response.ok) {
          data = await response.json();
          console.log('Received response from backend:', data);
        } else {
          throw new Error(`Backend API error: ${response.status} ${response.statusText}`);
        }
      } catch (error) {
        console.error('Error connecting to backend:', error);
        // Return error message to user
        const errorMessage: Message = {
          id: `msg-${Date.now() + 1}`,
          content: `${t('Connection error')}: ${error instanceof Error ? error.message : t('Failed to connect to backend API')}`,
          sender: 'assistant',
          timestamp: new Date(),
        };
        setMessages(prev => [...prev, errorMessage]);
        setIsLoading(false);
        return;
      }

      // Add assistant message
      const assistantMessage: Message = {
        id: `msg-${Date.now() + 1}`,
        content: data?.response || t("I'm having trouble processing your request. Please try again."),
        sender: 'assistant',
        timestamp: new Date(),
        citations: data?.citations || [],
      };

      // Add both user and assistant messages to the conversation
      setMessages(prev => [...prev, userMessage, assistantMessage]);
    } catch (error) {
      console.error('Chat error:', error);

      const errorMessage: Message = {
        id: `msg-${Date.now() + 1}`,
        content: `Error: ${error instanceof Error ? error.message : 'An unexpected error occurred'}`,
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
        <small>{t('Sources')}: {citations.map((c, index) =>
          `${c?.chapter_title || t('Unknown')}`
        ).join(', ')}</small>
      </div>
    );
  };

  return (
    <div className={styles.chatWidget}>
      {/* Selection button that appears when text is selected */}
      {showSelectionButton && (
        <button
          onClick={handleSelectionQuery}
          className={styles.selectionButton}
          style={{
            position: 'fixed',
            left: `${selectionPosition.x + 10}px`,
            top: `${selectionPosition.y - 40}px`,
            zIndex: 10000,
            backgroundColor: '#2563eb',
            color: 'white',
            border: 'none',
            borderRadius: '50%',
            width: '40px',
            height: '40px',
            cursor: 'pointer',
            boxShadow: '0 2px 10px rgba(0,0,0,0.2)',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            fontSize: '16px'
          }}
        >
          🔍
        </button>
      )}

      {isOpen ? (
        <div className={styles.chatContainer}>
          <div className={styles.chatHeader}>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', width: '100%' }}>
              <h3>{t('Physical AI Assistant')}</h3>
              <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
                <button onClick={toggleChat} className={styles.closeButton} title={t('Close')}>
                  ×
                </button>
              </div>
            </div>
          </div>

          <div className={styles.chatMessages}>
            {messages.length === 0 ? (
              <div className={styles.welcomeMessage}>
                <p>{t("Hello! I'm your Physical AI and Robotics Learning Assistant.")}</p>
                <p>{t("Ask me anything about the textbook content:")}</p>
                <ul>
                  <li>{t('ROS 2 fundamentals')}</li>
                  <li>{t('Simulation environments')}</li>
                  <li>{t('NVIDIA Isaac integration')}</li>
                  <li>{t('Vision-Language-Action systems')}</li>
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
              placeholder={t("Ask about the textbook content...")}
              disabled={isLoading}
              className={styles.chatInput}
            />
            <button
              type="submit"
              disabled={isLoading || !inputValue.trim()}
              className={styles.sendButton}
            >
              {t('Send')}
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