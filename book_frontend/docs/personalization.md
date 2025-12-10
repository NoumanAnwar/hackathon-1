---
id: personalization
title: User Authentication & Personalization
sidebar_position: 100
---

# User Authentication & Personalization

This section describes the implementation of user authentication and content personalization features using Better-Auth and advanced personalization capabilities.

## Better-Auth Integration

Better-Auth is implemented for secure user authentication and management:

### 1. Setup and Installation
```bash
npm install better-auth
```

### 2. Configuration
```javascript
// src/auth/better-auth-config.js
import { betterAuth } from "better-auth";

export const auth = betterAuth({
  database: {
    provider: "postgresql",
    url: process.env.DATABASE_URL,
  },
  socialProviders: {
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID,
      clientSecret: process.env.GOOGLE_CLIENT_SECRET,
    },
    github: {
      clientId: process.env.GITHUB_CLIENT_ID,
      clientSecret: process.env.GITHUB_CLIENT_SECRET,
    },
  },
  // Additional configuration options
});
```

### 3. User Registration Process
During registration, users are asked about their software and hardware background:

```javascript
// Form during registration
const registrationQuestions = [
  {
    id: "software_background",
    question: "What is your software development background?",
    type: "select",
    options: [
      "Beginner - Little to no experience",
      "Intermediate - Some programming experience", 
      "Advanced - Experienced developer",
      "Expert - Senior engineer/researcher"
    ]
  },
  {
    id: "hardware_background",
    question: "What is your hardware/robotics experience?",
    type: "select",
    options: [
      "Novice - No experience",
      "Basic - Hobby projects",
      "Intermediate - Academic projects",
      "Advanced - Professional experience"
    ]
  },
  {
    id: "primary_interest",
    question: "What interests you most about Physical AI?",
    type: "select",
    options: [
      "ROS 2 and robot middleware",
      "Simulation and digital twins",
      "AI and machine learning for robots",
      "Humanoid robotics applications"
    ]
  }
];
```

## Content Personalization

Authenticated users can personalize content in multiple ways:

### 1. Chapter Content Adaptation
Based on user background, chapter content is adapted:

```jsx
// Example personalization component
import { useAuth } from "better-auth/react";

const PersonalizedContent = ({ content }) => {
  const { session } = useAuth();
  
  if (!session?.user) {
    // Show basic content for unauthenticated users
    return <BasicContent content={content} />;
  }
  
  // Adapt content based on user profile
  switch(session.user.softwareBackground) {
    case "Beginner":
      return <BeginnerContent content={content} />;
    case "Intermediate":
      return <IntermediateContent content={content} />;
    case "Advanced":
      return <AdvancedContent content={content} />;
    default:
      return <DefaultContent content={content} />;
  }
};
```

### 2. Personalization Button Implementation
Each chapter includes a personalization button:

```jsx
// src/components/ContentPersonalizer.js
import React, { useState } from 'react';

const ContentPersonalizer = () => {
  const [isPersonalized, setIsPersonalized] = useState(false);
  
  const handlePersonalize = async () => {
    try {
      // Call personalization API endpoint
      await fetch('/api/personalize-content', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          userId: getAuthenticatedUserId(),
          chapterId: getCurrentChapterId(),
          preferences: getUserPreferences()
        })
      });
      
      setIsPersonalized(true);
      setTimeout(() => setIsPersonalized(false), 2000);
    } catch (error) {
      console.error('Error personalizing content:', error);
    }
  };
  
  return (
    <div className="personalization-controls">
      <button 
        onClick={handlePersonalize}
        className="personalize-btn"
        title="Personalize this content based on your profile"
      >
        🎯 Personalize Content
      </button>
      {isPersonalized && <span className="success-message">Content personalized!</span>}
    </div>
  );
};
```

## Urdu Translation Feature

Authenticated users can translate content to Urdu:

### 1. Translation Implementation
```jsx
// src/components/UrduTranslator.js
import React, { useState } from 'react';

const UrduTranslator = ({ content }) => {
  const [isTranslated, setIsTranslated] = useState(false);
  const [translatedContent, setTranslatedContent] = useState(null);
  
  const translateToUrdu = async () => {
    try {
      const response = await fetch('/api/translate', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          text: content,
          targetLanguage: 'ur'
        })
      });
      
      const result = await response.json();
      setTranslatedContent(result.translatedText);
      setIsTranslated(true);
    } catch (error) {
      console.error('Translation error:', error);
    }
  };
  
  return (
    <div className="translation-controls">
      <button 
        onClick={translateToUrdu}
        className="translate-btn"
        title="Translate to Urdu"
      >
        🇵🇰 Urdu
      </button>
      {isTranslated && (
        <div className="translated-content">
          <h4>اردو ترجمہ / Urdu Translation</h4>
          <p>{translatedContent}</p>
        </div>
      )}
    </div>
  );
};
```

### 2. Integration with Chapters
Each chapter page includes both personalization and translation controls:

```jsx
// Example chapter page structure
import ContentPersonalizer from '../components/ContentPersonalizer';
import UrduTranslator from '../components/UrduTranslator';

const ChapterPage = ({ content }) => {
  return (
    <article>
      <header>
        <h1>Chapter Title</h1>
        <div className="chapter-controls">
          <ContentPersonalizer />
          <UrduTranslator content={content} />
        </div>
      </header>
      <main>
        {/* Chapter content here */}
        {content}
      </main>
    </article>
  );
};
```

## User Profile Management

Users can manage their profile information:

### 1. Profile Update Interface
```jsx
// src/components/UserProfile.js
import { useAuth } from "better-auth/react";

const UserProfile = () => {
  const { session, signIn, signOut } = useAuth();
  const [profile, setProfile] = useState(session?.user || {});
  
  const updateProfile = async (updatedFields) => {
    try {
      await fetch('/api/user-profile', {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(updatedFields)
      });
      
      // Update session
      updateSession();
    } catch (error) {
      console.error('Profile update error:', error);
    }
  };
  
  if (!session) {
    return <div>Please sign in to access personalization features</div>;
  }
  
  return (
    <div className="user-profile">
      <h2>User Profile</h2>
      <div className="profile-info">
        <p>Software Background: {profile.softwareBackground}</p>
        <p>Hardware Background: {profile.hardwareBackground}</p>
        <p>Primary Interest: {profile.primaryInterest}</p>
      </div>
    </div>
  );
};
```

## Backend API Endpoints

### 1. Personalization API
```javascript
// api/personalize-content endpoint
app.post('/api/personalize-content', authenticateUser, async (req, res) => {
  const { userId, chapterId, preferences } = req.body;
  
  // Retrieve user profile
  const user = await getUserProfile(userId);
  
  // Apply personalization logic based on user profile
  const personalizedContent = await generatePersonalizedContent(
    chapterId, 
    user.background, 
    user.preferences
  );
  
  res.json({ content: personalizedContent });
});
```

### 2. Translation API
```javascript
// api/translate endpoint
app.post('/api/translate', authenticateUser, async (req, res) => {
  const { text, targetLanguage } = req.body;
  
  try {
    // Use translation service (e.g., Google Cloud Translation)
    const translatedText = await translateService.translate(
      text, 
      { target: targetLanguage }
    );
    
    res.json({ translatedText });
  } catch (error) {
    res.status(500).json({ error: 'Translation failed' });
  }
});
```

## Security and Privacy

- User data is encrypted and stored securely
- Personalization preferences are kept private
- Translation requests don't store user content permanently
- Authentication required for all personalization features

## Technical Implementation Notes

### 1. Component Integration
The personalization and translation components are integrated into the Docusaurus theme:

```javascript
// In theme configuration
module.exports = {
  // ... other config
  plugins: [
    // ... other plugins
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'personalization-plugin',
        path: 'src/components',
        routeBasePath: 'personalization',
      },
    ],
  ],
};
```

### 2. State Management
User preferences are stored and managed efficiently:

```javascript
// Using React Context for preference management
const PersonalizationContext = createContext();

export const PersonalizationProvider = ({ children }) => {
  const [preferences, setPreferences] = useState({});
  
  return (
    <PersonalizationContext.Provider value={{ preferences, setPreferences }}>
      {children}
    </PersonalizationContext.Provider>
  );
};
```

---

These personalization features provide users with a tailored learning experience based on their background and language preferences, meeting the requirements for signup, personalization, and Urdu translation capabilities.