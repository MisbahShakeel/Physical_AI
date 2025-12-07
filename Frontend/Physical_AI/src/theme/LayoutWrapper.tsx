import React from 'react';
import Layout from '@theme/Layout';
import ChatWidget from '../components/ChatWidget';

export default function LayoutWrapper(props) {
  return (
    <Layout {...props}>
      {props.children}
      <ChatWidget />
    </Layout>
  );
}