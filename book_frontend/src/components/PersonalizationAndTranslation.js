import React, { useState, useEffect } from 'react';

const PersonalizationAndTranslation = () => {
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [isTranslated, setIsTranslated] = useState(false);
  const [urduContent, setUrduContent] = useState('');
  const [showSuccessMessage, setShowSuccessMessage] = useState(false);
  const [showTranslation, setShowTranslation] = useState(false);

  // Check if user is authenticated (simplified check)
  const isAuthenticated = () => {
    // In a real implementation, this would check the Better-Auth session
    return typeof window !== 'undefined' && localStorage.getItem('user_authenticated') === 'true';
  };

  const handlePersonalize = async () => {
    if (!isAuthenticated()) {
      alert('Please sign in to use personalization features');
      return;
    }

    try {
      // Simulate API call to personalize content
      // In a real implementation, this would call the backend
      await new Promise(resolve => setTimeout(resolve, 500)); // Simulate API call
      
      setIsPersonalized(true);
      setShowSuccessMessage(true);
      
      // Hide success message after 2 seconds
      setTimeout(() => {
        setShowSuccessMessage(false);
      }, 2000);
      
    } catch (error) {
      console.error('Error personalizing content:', error);
      alert('Error personalizing content. Please try again.');
    }
  };

  const handleTranslateToUrdu = async () => {
    if (!isAuthenticated()) {
      alert('Please sign in to use translation features');
      return;
    }

    try {
      // Simulate API call to translate content
      // In a real implementation, this would call the translation API
      await new Promise(resolve => setTimeout(resolve, 800)); // Simulate API call
      
      // Mock Urdu translation
      setUrduContent('یہ مشینی حفاظتی سیکھنے کی ایک مثال ہے۔ ڈیٹا کے نمونوں کی کم تعداد پر تربیت یافتہ ماڈل کو بے نقاب کر دیا جائے گا۔');
      setIsTranslated(true);
      setShowTranslation(true);
      
    } catch (error) {
      console.error('Error translating content:', error);
      alert('Error translating content. Please try again.');
    }
  };

  return (
    <div className="personalization-translation-container" style={{ 
      display: 'flex', 
      gap: '10px', 
      margin: '20px 0',
      padding: '10px',
      border: '1px solid var(--ifm-color-emphasis-300)',
      borderRadius: 'var(--ifm-border-radius)',
      backgroundColor: 'var(--ifm-color-emphasis-100)'
    }}>
      <button 
        onClick={handlePersonalize}
        disabled={!isAuthenticated()}
        style={{
          padding: '8px 12px',
          borderRadius: 'var(--ifm-border-radius)',
          border: 'none',
          backgroundColor: isPersonalized 
            ? 'var(--ifm-color-success)' 
            : 'var(--ifm-color-emphasis-200)',
          color: 'var(--ifm-color-emphasis-1000)',
          cursor: isAuthenticated() ? 'pointer' : 'not-allowed',
          display: 'flex',
          alignItems: 'center',
          gap: '5px'
        }}
        title={isAuthenticated() 
          ? "Personalize this content based on your profile" 
          : "Sign in to use personalization features"}
      >
        🎯 Personalize
        {isPersonalized && (
          <span style={{ fontSize: '0.8em' }}>Applied</span>
        )}
      </button>

      <button 
        onClick={handleTranslateToUrdu}
        disabled={!isAuthenticated()}
        style={{
          padding: '8px 12px',
          borderRadius: 'var(--ifm-border-radius)',
          border: 'none',
          backgroundColor: isTranslated 
            ? 'var(--ifm-color-info)' 
            : 'var(--ifm-color-emphasis-200)',
          color: 'var(--ifm-color-emphasis-1000)',
          cursor: isAuthenticated() ? 'pointer' : 'not-allowed',
          display: 'flex',
          alignItems: 'center',
          gap: '5px'
        }}
        title={isAuthenticated() 
          ? "Translate to Urdu" 
          : "Sign in to use translation features"}
      >
        🇵🇰 UR
      </button>

      {showSuccessMessage && (
        <div style={{
          position: 'fixed',
          top: '20px',
          right: '20px',
          backgroundColor: 'var(--ifm-color-success)',
          color: 'white',
          padding: '10px 15px',
          borderRadius: 'var(--ifm-border-radius)',
          zIndex: 1000
        }}>
          Content personalized successfully!
        </div>
      )}

      {showTranslation && (
        <div style={{
          marginTop: '15px',
          padding: '10px',
          backgroundColor: 'var(--ifm-color-info-lightest)',
          border: '1px solid var(--ifm-color-info)',
          borderRadius: 'var(--ifm-border-radius)',
          width: '100%'
        }}>
          <h4 style={{ margin: '0 0 10px 0', color: 'var(--ifm-color-info-dark)' }}>اردو ترجمہ / Urdu Translation:</h4>
          <p style={{ margin: 0 }}>{urduContent}</p>
        </div>
      )}

      {!isAuthenticated() && (
        <div style={{
          marginLeft: '10px',
          fontSize: '0.8em',
          color: 'var(--ifm-color-emphasis-600)',
          fontStyle: 'italic'
        }}>
          Sign in to unlock personalization and translation features
        </div>
      )}
    </div>
  );
};

export default PersonalizationAndTranslation;