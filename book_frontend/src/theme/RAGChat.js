import React from 'react';
import Chatbox from '@site/src/components/Chatbox';

// Wrapper component for the RAG chatbot that can be used in MDX files
const RAGChat = (props) => {
  return (
    <div style={{ 
      margin: '20px 0',
      padding: '15px',
      border: '1px solid var(--ifm-color-primary-light)',
      borderRadius: 'var(--ifm-border-radius)',
      backgroundColor: 'var(--ifm-color-emphasis-100)'
    }}>
      <h3>Ask the RAG Chatbot</h3>
      <p style={{ fontSize: '0.9em', color: 'var(--ifm-color-emphasis-700)', marginBottom: '15px' }}>
        Ask questions about this content and get AI-powered answers based on the book's content:
      </p>
      <Chatbox {...props} />
    </div>
  );
};

export default RAGChat;