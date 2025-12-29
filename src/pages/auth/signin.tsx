/**
 * Signin Page
 *
 * Route: /auth/signin
 */

import React from 'react';
import Layout from '@theme/Layout';
import SigninForm from '@site/src/components/Auth/SigninForm';

export default function SigninPage(): JSX.Element {
  const handleSuccess = (user: any) => {
    // Redirect to home page after successful signin
    window.location.href = '/';
  };

  return (
    <Layout
      title="Sign In"
      description="Sign in to access personalized learning features"
      noFooter
    >
      <SigninForm onSuccess={handleSuccess} />
    </Layout>
  );
}
