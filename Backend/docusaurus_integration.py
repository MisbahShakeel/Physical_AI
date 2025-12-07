"""
Docusaurus Chat Widget Integration for Textbook RAG System
This file contains the backend logic to support the frontend chat widget
"""
from fastapi import APIRouter, Request
from pydantic import BaseModel
from typing import Dict, Any, Optional
import json
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Router for Docusaurus-specific endpoints
docusaurus_router = APIRouter(prefix="/docusaurus", tags=["docusaurus"])

class ChatWidgetConfig(BaseModel):
    """Configuration for the chat widget"""
    api_base_url: str
    widget_title: str = "Textbook Assistant"
    initial_message: str = "Ask questions about the textbook content!"
    enable_selected_text: bool = True
    max_history_messages: int = 10
    theme: str = "light"  # "light" or "dark"

class ChatWidgetResponse(BaseModel):
    """Response from chat widget endpoints"""
    status: str
    data: Optional[Dict[str, Any]] = None
    message: Optional[str] = None

@docusaurus_router.get("/widget-config")
async def get_widget_config() -> ChatWidgetConfig:
    """
    Get configuration for the Docusaurus chat widget
    This endpoint provides configuration that the frontend widget can use
    """
    config = ChatWidgetConfig(
        api_base_url="/api",  # Relative to the Docusaurus site
        widget_title="Textbook Assistant",
        initial_message="Ask questions about the textbook content!",
        enable_selected_text=True,
        max_history_messages=10,
        theme="light"
    )
    return config

@docusaurus_router.post("/page-context")
async def get_page_context(request: Request) -> Dict[str, Any]:
    """
    Get context information for the current page
    This helps the chat widget understand what content is currently visible
    """
    try:
        # Extract page information from request
        page_data = {
            "url": str(request.url),
            "path": request.url.path,
            "title": "Textbook Page",  # This would come from page metadata
            "section": "Unknown",      # This would be extracted from URL or metadata
            "last_updated": "2024-01-01"  # This would come from page metadata
        }

        return {"status": "success", "data": page_data}
    except Exception as e:
        logger.error(f"Error getting page context: {str(e)}")
        return {"status": "error", "message": str(e)}

@docusaurus_router.get("/widget-styles")
async def get_widget_styles() -> Dict[str, str]:
    """
    Get CSS styles for the chat widget
    This allows for theme customization based on Docusaurus site
    """
    styles = {
        "primary_color": "#2563eb",
        "secondary_color": "#f8fafc",
        "text_color": "#1e293b",
        "background_color": "#ffffff",
        "border_radius": "8px",
        "font_family": "system-ui, -apple-system, sans-serif",
        "widget_position": "bottom-right",
        "widget_size": "400x600"  # width x height in pixels
    }
    return styles

# Additional integration functions
async def inject_chat_widget_script(base_url: str) -> str:
    """
    Generate the script tag to inject into Docusaurus pages
    This would be used in Docusaurus configuration to add the chat widget
    """
    script_content = f"""
    <!-- Textbook RAG Chat Widget -->
    <script>
    (function() {{
        // Create and inject the chat widget
        const widgetScript = document.createElement('script');
        widgetScript.src = '{base_url}/static/chat-widget.js';
        widgetScript.async = true;
        document.head.appendChild(widgetScript);

        // Add widget styles
        const widgetStyles = document.createElement('link');
        widgetStyles.rel = 'stylesheet';
        widgetStyles.href = '{base_url}/static/chat-widget.css';
        document.head.appendChild(widgetStyles);
    }})();
    </script>
    """
    return script_content

async def get_docusaurus_config_snippet() -> str:
    """
    Get the configuration snippet to add to Docusaurus config
    This helps users integrate the widget into their Docusaurus site
    """
    config_snippet = """
    // In your docusaurus.config.js
    module.exports = {
      // ... other config
      scripts: [
        // ... other scripts
        {
          src: '/api/docusaurus/widget-script',
          async: true,
          defer: true
        }
      ],
      stylesheets: [
        // ... other stylesheets
        {
          href: '/api/docusaurus/widget-styles.css',
          type: 'text/css'
        }
      ]
    };
    """
    return config_snippet

# Frontend assets would typically be served from a static directory
# For now, we'll define them as functions that could generate the content

def generate_chat_widget_js() -> str:
    """
    Generate the JavaScript for the chat widget
    This would be served as a static file in a real implementation
    """
    js_content = """
    // Textbook Chat Widget JavaScript
    class TextbookChatWidget {
        constructor(config) {
            this.config = config || {};
            this.isOpen = false;
            this.messages = [];
            this.sessionId = this.generateSessionId();
            this.init();
        }

        init() {
            this.createWidget();
            this.loadStyles();
            this.setupEventListeners();
        }

        createWidget() {
            // Create widget container
            this.widget = document.createElement('div');
            this.widget.className = 'textbook-chat-widget';
            this.widget.innerHTML = `
                <div class="chat-container" style="display: none;">
                    <div class="chat-header">
                        <h3>${this.config.widget_title || 'Textbook Assistant'}</h3>
                        <button class="close-btn" onclick="chatWidget.toggle()">×</button>
                    </div>
                    <div class="chat-messages"></div>
                    <form class="chat-input-form">
                        <input type="text" class="chat-input" placeholder="Ask about the textbook content..." />
                        <button type="submit">Send</button>
                    </form>
                </div>
                <button class="chat-toggle-btn">💬</button>
            `;
            document.body.appendChild(this.widget);

            // Cache DOM elements
            this.chatContainer = this.widget.querySelector('.chat-container');
            this.chatMessages = this.widget.querySelector('.chat-messages');
            this.chatInput = this.widget.querySelector('.chat-input');
            this.chatForm = this.widget.querySelector('.chat-input-form');
            this.toggleBtn = this.widget.querySelector('.chat-toggle-btn');
        }

        loadStyles() {
            // Create and inject styles
            const style = document.createElement('style');
            style.textContent = `
                .textbook-chat-widget {
                    position: fixed;
                    bottom: 20px;
                    right: 20px;
                    z-index: 1000;
                }

                .chat-container {
                    width: 400px;
                    height: 600px;
                    border: 1px solid #ddd;
                    border-radius: 8px;
                    background: white;
                    display: flex;
                    flex-direction: column;
                    box-shadow: 0 4px 12px rgba(0,0,0,0.15);
                }

                .chat-header {
                    padding: 15px;
                    background: #2563eb;
                    color: white;
                    border-radius: 8px 8px 0 0;
                    display: flex;
                    justify-content: space-between;
                    align-items: center;
                }

                .close-btn {
                    background: none;
                    border: none;
                    color: white;
                    font-size: 20px;
                    cursor: pointer;
                }

                .chat-messages {
                    flex: 1;
                    overflow-y: auto;
                    padding: 15px;
                }

                .message {
                    margin-bottom: 15px;
                    max-width: 80%;
                }

                .user-message {
                    text-align: right;
                    margin-left: 20%;
                }

                .user-message .content {
                    display: inline-block;
                    background: #e0f2fe;
                    padding: 10px 15px;
                    border-radius: 18px 18px 4px 18px;
                }

                .assistant-message .content {
                    display: inline-block;
                    background: #f8fafc;
                    padding: 10px 15px;
                    border-radius: 18px 18px 18px 4px;
                    border-left: 3px solid #2563eb;
                }

                .chat-input-form {
                    padding: 15px;
                    border-top: 1px solid #eee;
                    display: flex;
                }

                .chat-input {
                    flex: 1;
                    padding: 10px;
                    border: 1px solid #ddd;
                    border-radius: 4px;
                    margin-right: 10px;
                }

                .chat-input-form button {
                    padding: 10px 15px;
                    background: #2563eb;
                    color: white;
                    border: none;
                    border-radius: 4px;
                    cursor: pointer;
                }

                .chat-toggle-btn {
                    width: 60px;
                    height: 60px;
                    border-radius: 50%;
                    background: #2563eb;
                    color: white;
                    border: none;
                    font-size: 24px;
                    cursor: pointer;
                    box-shadow: 0 4px 12px rgba(0,0,0,0.15);
                    display: flex;
                    align-items: center;
                    justify-content: center;
                }
            `;
            document.head.appendChild(style);
        }

        setupEventListeners() {
            // Toggle button
            this.toggleBtn.addEventListener('click', () => this.toggle());

            // Form submission
            this.chatForm.addEventListener('submit', (e) => {
                e.preventDefault();
                this.sendMessage();
            });

            // Text selection
            document.addEventListener('mouseup', () => {
                const selection = window.getSelection();
                if (selection.toString().trim().length > 0) {
                    this.selectedText = selection.toString();
                }
            });
        }

        toggle() {
            this.chatContainer.style.display = this.isOpen ? 'none' : 'flex';
            this.isOpen = !this.isOpen;
        }

        async sendMessage() {
            const message = this.chatInput.value.trim();
            if (!message) return;

            // Add user message to UI
            this.addMessage(message, 'user');
            this.chatInput.value = '';

            try {
                // Get current page context
                const pageContext = await this.getPageContext();

                // Prepare request payload
                const payload = {
                    query: message,
                    session_id: this.sessionId,
                    selected_text: this.selectedText || null
                };

                // Determine endpoint based on selected text
                const endpoint = this.selectedText ? '/api/query-selected' : '/api/query';

                // Send request to backend
                const response = await fetch(endpoint, {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify(payload)
                });

                if (!response.ok) {
                    throw new Error(`API error: ${response.status}`);
                }

                const data = await response.json();

                // Add assistant response to UI
                this.addMessage(data.response, 'assistant', data.citations);

                // Clear selected text after query
                this.selectedText = '';

            } catch (error) {
                console.error('Chat error:', error);
                this.addMessage('Sorry, I encountered an error. Please try again.', 'assistant');
            }
        }

        addMessage(content, sender, citations = null) {
            const messageDiv = document.createElement('div');
            messageDiv.className = `message ${sender}-message`;

            const contentDiv = document.createElement('div');
            contentDiv.className = 'content';
            contentDiv.textContent = content;

            messageDiv.appendChild(contentDiv);

            // Add citations if provided
            if (citations && citations.length > 0) {
                const citationsDiv = document.createElement('div');
                citationsDiv.className = 'citations';
                citationsDiv.style.fontSize = '0.8em';
                citationsDiv.style.color = '#64748b';
                citationsDiv.style.marginTop = '5px';

                const citationText = document.createElement('small');
                citationText.textContent = 'Sources: ' + citations.map(c =>
                    `${c.chapter_title} - ${c.section_title}`
                ).join(', ');

                citationsDiv.appendChild(citationText);
                messageDiv.appendChild(citationsDiv);
            }

            this.chatMessages.appendChild(messageDiv);
            this.chatMessages.scrollTop = this.chatMessages.scrollHeight;
        }

        async getPageContext() {
            try {
                const response = await fetch('/api/docusaurus/page-context');
                if (response.ok) {
                    return await response.json();
                }
            } catch (error) {
                console.error('Error getting page context:', error);
            }
            return {};
        }

        generateSessionId() {
            return 'session_' + Date.now() + '_' + Math.random().toString(36).substr(2, 9);
        }
    }

    // Initialize the chat widget when the page loads
    document.addEventListener('DOMContentLoaded', function() {
        // Fetch widget configuration
        fetch('/api/docusaurus/widget-config')
            .then(response => response.json())
            .then(config => {
                window.chatWidget = new TextbookChatWidget(config);
            })
            .catch(error => {
                console.error('Error loading chat widget config:', error);
                // Initialize with default config
                window.chatWidget = new TextbookChatWidget({});
            });
    });
    """
    return js_content

def generate_chat_widget_css() -> str:
    """
    Generate the CSS for the chat widget
    This would be served as a static file in a real implementation
    """
    css_content = """
    /* Textbook Chat Widget Styles */
    .textbook-chat-widget {
        position: fixed;
        bottom: 20px;
        right: 20px;
        z-index: 1000;
    }

    .chat-container {
        width: 400px;
        height: 600px;
        border: 1px solid #ddd;
        border-radius: 8px;
        background: white;
        display: flex;
        flex-direction: column;
        box-shadow: 0 4px 12px rgba(0,0,0,0.15);
    }

    .chat-header {
        padding: 15px;
        background: #2563eb;
        color: white;
        border-radius: 8px 8px 0 0;
        display: flex;
        justify-content: space-between;
        align-items: center;
    }

    .close-btn {
        background: none;
        border: none;
        color: white;
        font-size: 20px;
        cursor: pointer;
    }

    .chat-messages {
        flex: 1;
        overflow-y: auto;
        padding: 15px;
    }

    .message {
        margin-bottom: 15px;
    }

    .user-message {
        text-align: right;
    }

    .user-message .content {
        display: inline-block;
        background: #e0f2fe;
        padding: 10px 15px;
        border-radius: 18px 18px 4px 18px;
        max-width: 80%;
    }

    .assistant-message {
        text-align: left;
    }

    .assistant-message .content {
        display: inline-block;
        background: #f8fafc;
        padding: 10px 15px;
        border-radius: 18px 18px 18px 4px;
        max-width: 80%;
        border-left: 3px solid #2563eb;
    }

    .citations {
        margin-top: 8px;
        font-size: 0.8em;
        color: #64748b;
    }

    .citation {
        background: #e2e8f0;
        padding: 2px 6px;
        border-radius: 12px;
        margin-right: 4px;
    }

    .chat-input-form {
        padding: 15px;
        border-top: 1px solid #eee;
        display: flex;
    }

    .chat-input {
        flex: 1;
        padding: 10px;
        border: 1px solid #ddd;
        border-radius: 4px;
        margin-right: 10px;
    }

    .chat-input-form button {
        padding: 10px 15px;
        background: #2563eb;
        color: white;
        border: none;
        border-radius: 4px;
        cursor: pointer;
    }

    .chat-input-form button:disabled {
        background: #94a3b8;
        cursor: not-allowed;
    }

    .chat-toggle-btn {
        width: 60px;
        height: 60px;
        border-radius: 50%;
        background: #2563eb;
        color: white;
        border: none;
        font-size: 24px;
        cursor: pointer;
        box-shadow: 0 4px 12px rgba(0,0,0,0.15);
        display: flex;
        align-items: center;
        justify-content: center;
    }

    .chat-toggle-btn.hidden {
        display: none;
    }

    .typing-indicator {
        display: inline-block;
        padding: 10px 15px;
        background: #f8fafc;
        border-radius: 18px 18px 18px 4px;
    }

    .typing-indicator span {
        display: inline-block;
        width: 8px;
        height: 8px;
        border-radius: 50%;
        background: #94a3b8;
        margin: 0 2px;
        animation: typing 1.4s infinite ease-in-out;
    }

    .typing-indicator span:nth-child(1) { animation-delay: -0.32s; }
    .typing-indicator span:nth-child(2) { animation-delay: -0.16s; }

    @keyframes typing {
        0%, 80%, 100% { transform: scale(0); }
        40% { transform: scale(1.0); }
    }
    """
    return css_content

# Export the router
__all__ = ['docusaurus_router']