import React from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import ChatWidget from '@site/src/components/ChatWidget';

export default function Home() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Your all-in-one AI book for learning and experimentation"
    >
        <main>
        <HomepageFeatures />
      </main>

      {/* Chat widget rendered on all pages */}
      <ChatWidget />
    </Layout>
  );
}
