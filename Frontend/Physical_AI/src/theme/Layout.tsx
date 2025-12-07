import React, { useEffect } from 'react';
import Layout from '@theme/Layout';
import ChatWidget from '@site/src/components/ChatWidget';

export default function LayoutWrapper(props) {
  return (
    <>
      <Layout {...props}>
        {props.children}
      </Layout>
      <ChatWidget />
    </>
  );
}