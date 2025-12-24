import React from 'react';
import DocItemLayout from '@theme-original/DocItem/Layout';
import ChatWidget from '@site/src/components/ChatWidget';

export default function DocItemLayoutWrapper(props) {
  return (
    <>
      <DocItemLayout {...props} />
      <ChatWidget />
    </>
  );
}
