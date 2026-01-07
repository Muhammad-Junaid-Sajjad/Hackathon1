/**
 * Signup Page
 *
 * Route: /auth/signup
 */

import React from 'react';
import Layout from '@theme/Layout';
import SignupForm from '@site/src/components/Utils/SignupForm';

export default function SignupPage(): JSX.Element {
  const handleSuccess = (user: any) => {
    // Redirect to home page after successful signup
    window.location.href = '/';
  };

  return (
    <Layout
      title="Create Account"
      description="Join the Physical AI & Humanoid Robotics learning community"
      noFooter
    >
      <SignupForm onSuccess={handleSuccess} />
    </Layout>
  );
}
