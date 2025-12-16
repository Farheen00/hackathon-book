import React, { useState, useRef, useEffect } from 'react';
import './ChatUI.css';

interface Message {
  id: string;
  content: string;
  role: 'user' | 'assistant';
  timestamp: number;
}

interface ApiResponse {
  answer: string;
  sources: string[];
  matched_chunks: Array<{
    content: string;
    source_url: string;
    similarity_score: number;
  }>;
  processing_time: number;
  timestamp: number;
}

const ChatUI: React.FC = () => {
  // ✅ Client-side only
  if (typeof window === 'undefined') return null;

  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [isDarkTheme, setIsDarkTheme] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  // Determine theme based on system preference or document class
  useEffect(() => {
    const checkTheme = () => {
      const darkThemeClass = document.documentElement.classList.contains('dark-theme');
      const systemPrefersDark = window.matchMedia('(prefers-color-scheme: dark)').matches;
      setIsDarkTheme(darkThemeClass || systemPrefersDark);
    };

    // Check initial theme
    checkTheme();

    // Listen for theme changes
    const mediaQuery = window.matchMedia('(prefers-color-scheme: dark)');
    const handler = () => checkTheme();
    mediaQuery.addEventListener('change', handler);

    // Cleanup
    return () => {
      mediaQuery.removeEventListener('change', handler);
    };
  }, []);

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen && inputRef.current) {
      setTimeout(() => inputRef.current?.focus(), 100);
    }
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      content: inputValue,
      role: 'user',
      timestamp: Date.now(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch('http://localhost:8000/ask', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ query: inputValue, top_k: 5, temperature: 0.7 }),
      });

      if (!response.ok) throw new Error(`API error: ${response.status}`);

      const data: ApiResponse = await response.json();

      const assistantMessage: Message = {
        id: Date.now().toString(),
        content: formatResponse(data),
        role: 'assistant',
        timestamp: Date.now(),
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (err) {
      console.error(err);
      setError('Failed to get response. Please try again.');

      const errorMessage: Message = {
        id: Date.now().toString(),
        content: 'Sorry, I encountered an error processing your request.',
        role: 'assistant',
        timestamp: Date.now(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const formatResponse = (data: ApiResponse): string => {
    let response = data.answer;

    if (data.sources.length > 0) {
      response += '\n\n**Sources:**\n';
      data.sources.slice(0, 3).forEach(s => {
        response += `- [${s}](${s})\n`;
      });
    }

    if (data.matched_chunks.length > 0) {
      response += '\n**Relevant information:**\n';
      data.matched_chunks.slice(0, 2).forEach(c => {
        const preview = c.content.length > 100 ? c.content.substring(0, 100) + '...' : c.content;
        response += `- *${preview}*\n`;
      });
    }

    return response;
  };

  const clearChat = () => {
    setMessages([]);
    setError(null);
  };

  return (
    <>
      {!isOpen && (
        <button className={`chat-button ${isDarkTheme ? 'dark-theme' : 'light-theme'}`} onClick={toggleChat}>
          Chat
        </button>
      )}

      {isOpen && (
        <div className={`chat-container ${isDarkTheme ? 'dark-theme' : 'light-theme'}`}>
          <div className="chat-header">
            <h3>RAG Assistant</h3>
            <div className="chat-header-actions">
              <button className={`clear-chat-btn ${isDarkTheme ? 'dark-theme' : 'light-theme'}`} onClick={clearChat}>Clear</button>
              <button className={`close-chat-btn ${isDarkTheme ? 'dark-theme' : 'light-theme'}`} onClick={toggleChat}>Close</button>
            </div>
          </div>

          <div className="chat-messages">
            {messages.length === 0 && (
              <div className="chat-welcome">
                <p>Ask me anything about the documentation!</p>
              </div>
            )}
            {messages.map(m => (
              <div key={m.id} className={`message ${m.role} ${isDarkTheme ? 'dark-theme' : 'light-theme'}`}>
                {m.role === 'assistant' ? (
                  <div dangerouslySetInnerHTML={{ __html: formatMessageContent(m.content) }} />
                ) : (
                  <span>{m.content}</span>
                )}
              </div>
            ))}
            {isLoading && (
              <div className={`message assistant ${isDarkTheme ? 'dark-theme' : 'light-theme'}`}>
                <div className="typing-indicator">
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {error && <div className="chat-error">{error}</div>}

          <form onSubmit={handleSubmit} className="chat-input-form">
            <input
              ref={inputRef}
              value={inputValue}
              onChange={e => setInputValue(e.target.value)}
              placeholder="Type your message..."
              className={isDarkTheme ? 'dark-theme' : 'light-theme'}
            />
            <button type="submit" disabled={!inputValue.trim() || isLoading} className={isDarkTheme ? 'dark-theme' : 'light-theme'}>
              Send
            </button>
          </form>
        </div>
      )}
    </>
  );
};

const formatMessageContent = (content: string) =>
  content
    .replace(/\*\*(.*?)\*\*/g, '<strong>$1</strong>')
    .replace(/\*(.*?)\*/g, '<em>$1</em>')
    .replace(/\n/g, '<br />');

export default ChatUI;