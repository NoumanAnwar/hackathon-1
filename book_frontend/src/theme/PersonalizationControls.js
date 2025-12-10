import React from 'react';
import PersonalizationAndTranslation from '@site/src/components/PersonalizationAndTranslation';

// Wrapper component for personalization features that can be used in MDX files
const PersonalizationControls = (props) => {
  return (
    <div style={{ 
      margin: '20px 0',
      padding: '15px',
      border: '1px solid var(--ifm-color-info)',
      borderRadius: 'var(--ifm-border-radius)',
      backgroundColor: 'var(--ifm-color-emphasis-100)'
    }}>
      <h3>Personalize This Content</h3>
      <p style={{ fontSize: '0.9em', color: 'var(--ifm-color-emphasis-700)', marginBottom: '15px' }}>
        Sign in to unlock content personalization and Urdu translation features:
      </p>
      <PersonalizationAndTranslation {...props} />
    </div>
  );
};

export default PersonalizationControls;