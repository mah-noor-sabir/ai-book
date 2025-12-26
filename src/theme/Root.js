// src/theme/Root.js
import React from 'react';
import ChatWidget from '../components/ChatWidget'; // your existing ChatWidget

export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}
