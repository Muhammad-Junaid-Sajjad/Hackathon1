/**
 * Signup Form Component with Background Questions
 *
 * Requirements addressed:
 * - Requirement 5: Better-Auth signup with user background questions
 */

import React, { useState } from 'react';
import styles from './styles.module.css';

interface UserBackground {
  programmingExperience: string;
  roboticsExperience: string;
  hardwareAccess: string[];
  learningGoals: string[];
  preferredDepth: string;
}

interface SignupFormProps {
  onSuccess?: (user: any) => void;
  apiUrl?: string;
}

const HARDWARE_OPTIONS = [
  { id: 'jetson-nano', label: 'NVIDIA Jetson Nano' },
  { id: 'jetson-orin', label: 'NVIDIA Jetson Orin' },
  { id: 'realsense', label: 'Intel RealSense Camera' },
  { id: 'lidar', label: 'LiDAR Sensor' },
  { id: 'robot-arm', label: 'Robot Arm (any)' },
  { id: 'mobile-robot', label: 'Mobile Robot Base' },
  { id: 'simulation-only', label: 'Simulation Only (No Hardware)' },
];

const LEARNING_GOALS = [
  { id: 'ros2', label: 'Learn ROS 2 Development' },
  { id: 'perception', label: 'Build Perception Systems' },
  { id: 'manipulation', label: 'Robot Manipulation' },
  { id: 'locomotion', label: 'Humanoid Locomotion' },
  { id: 'integration', label: 'Full System Integration' },
  { id: 'research', label: 'Research & Innovation' },
  { id: 'career', label: 'Career in Robotics' },
];

const SignupForm: React.FC<SignupFormProps> = ({ onSuccess, apiUrl = '' }) => {
  const [step, setStep] = useState(1);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Form data
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    confirmPassword: '',
    name: '',
  });

  const [background, setBackground] = useState<UserBackground>({
    programmingExperience: '',
    roboticsExperience: '',
    hardwareAccess: [],
    learningGoals: [],
    preferredDepth: 'intermediate',
  });

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    setFormData(prev => ({
      ...prev,
      [e.target.name]: e.target.value,
    }));
  };

  const handleSelectChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    setBackground(prev => ({
      ...prev,
      [e.target.name]: e.target.value,
    }));
  };

  const handleCheckboxChange = (field: 'hardwareAccess' | 'learningGoals', value: string) => {
    setBackground(prev => ({
      ...prev,
      [field]: prev[field].includes(value)
        ? prev[field].filter(v => v !== value)
        : [...prev[field], value],
    }));
  };

  const validateStep1 = () => {
    if (!formData.email || !formData.password || !formData.name) {
      setError('Please fill in all fields');
      return false;
    }
    if (formData.password.length < 8) {
      setError('Password must be at least 8 characters');
      return false;
    }
    if (formData.password !== formData.confirmPassword) {
      setError('Passwords do not match');
      return false;
    }
    return true;
  };

  const validateStep2 = () => {
    if (!background.programmingExperience || !background.roboticsExperience) {
      setError('Please answer all required questions');
      return false;
    }
    return true;
  };

  const handleNext = () => {
    setError(null);
    if (step === 1 && validateStep1()) {
      setStep(2);
    } else if (step === 2 && validateStep2()) {
      setStep(3);
    }
  };

  const handleBack = () => {
    setError(null);
    setStep(prev => prev - 1);
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setIsLoading(true);

    try {
      const response = await fetch(`${apiUrl}/api/auth/signup`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          email: formData.email,
          password: formData.password,
          name: formData.name,
          background: {
            programming_experience: background.programmingExperience,
            robotics_experience: background.roboticsExperience,
            hardware_access: background.hardwareAccess,
            learning_goals: background.learningGoals,
            preferred_depth: background.preferredDepth,
          },
        }),
      });

      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.message || 'Signup failed');
      }

      const data = await response.json();
      onSuccess?.(data.user);
    } catch (err: any) {
      setError(err.message || 'An error occurred during signup');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.authContainer}>
      <div className={styles.authCard}>
        <div className={styles.header}>
          <h2>Create Your Account</h2>
          <p>Join the Physical AI & Humanoid Robotics learning community</p>
        </div>

        {/* Progress Steps */}
        <div className={styles.progressSteps}>
          <div className={`${styles.step} ${step >= 1 ? styles.active : ''}`}>
            <span>1</span>
            <label>Account</label>
          </div>
          <div className={styles.stepLine}></div>
          <div className={`${styles.step} ${step >= 2 ? styles.active : ''}`}>
            <span>2</span>
            <label>Experience</label>
          </div>
          <div className={styles.stepLine}></div>
          <div className={`${styles.step} ${step >= 3 ? styles.active : ''}`}>
            <span>3</span>
            <label>Goals</label>
          </div>
        </div>

        {error && (
          <div className={styles.errorAlert}>
            <span>⚠️</span> {error}
          </div>
        )}

        <form onSubmit={handleSubmit}>
          {/* Step 1: Account Info */}
          {step === 1 && (
            <div className={styles.formStep}>
              <div className={styles.formGroup}>
                <label htmlFor="name">Full Name</label>
                <input
                  type="text"
                  id="name"
                  name="name"
                  value={formData.name}
                  onChange={handleInputChange}
                  placeholder="Enter your full name"
                  required
                />
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="email">Email Address</label>
                <input
                  type="email"
                  id="email"
                  name="email"
                  value={formData.email}
                  onChange={handleInputChange}
                  placeholder="Enter your email"
                  required
                />
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="password">Password</label>
                <input
                  type="password"
                  id="password"
                  name="password"
                  value={formData.password}
                  onChange={handleInputChange}
                  placeholder="At least 8 characters"
                  required
                />
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="confirmPassword">Confirm Password</label>
                <input
                  type="password"
                  id="confirmPassword"
                  name="confirmPassword"
                  value={formData.confirmPassword}
                  onChange={handleInputChange}
                  placeholder="Confirm your password"
                  required
                />
              </div>

              <button type="button" className={styles.primaryButton} onClick={handleNext}>
                Continue →
              </button>
            </div>
          )}

          {/* Step 2: Background Experience */}
          {step === 2 && (
            <div className={styles.formStep}>
              <p className={styles.stepDescription}>
                Help us personalize your learning experience by telling us about your background.
              </p>

              <div className={styles.formGroup}>
                <label htmlFor="programmingExperience">Programming Experience *</label>
                <select
                  id="programmingExperience"
                  name="programmingExperience"
                  value={background.programmingExperience}
                  onChange={handleSelectChange}
                  required
                >
                  <option value="">Select your experience level</option>
                  <option value="none">No programming experience</option>
                  <option value="beginner">Beginner (learning basics)</option>
                  <option value="intermediate">Intermediate (comfortable with Python)</option>
                  <option value="advanced">Advanced (multiple languages, frameworks)</option>
                  <option value="expert">Expert (professional developer)</option>
                </select>
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="roboticsExperience">Robotics Experience *</label>
                <select
                  id="roboticsExperience"
                  name="roboticsExperience"
                  value={background.roboticsExperience}
                  onChange={handleSelectChange}
                  required
                >
                  <option value="">Select your experience level</option>
                  <option value="none">No robotics experience</option>
                  <option value="hobbyist">Hobbyist (Arduino, DIY projects)</option>
                  <option value="student">Student (coursework, labs)</option>
                  <option value="professional">Professional (industry experience)</option>
                  <option value="researcher">Researcher (academic/R&D)</option>
                </select>
              </div>

              <div className={styles.formGroup}>
                <label>Available Hardware (select all that apply)</label>
                <div className={styles.checkboxGroup}>
                  {HARDWARE_OPTIONS.map(option => (
                    <label key={option.id} className={styles.checkboxLabel}>
                      <input
                        type="checkbox"
                        checked={background.hardwareAccess.includes(option.id)}
                        onChange={() => handleCheckboxChange('hardwareAccess', option.id)}
                      />
                      <span>{option.label}</span>
                    </label>
                  ))}
                </div>
              </div>

              <div className={styles.buttonGroup}>
                <button type="button" className={styles.secondaryButton} onClick={handleBack}>
                  ← Back
                </button>
                <button type="button" className={styles.primaryButton} onClick={handleNext}>
                  Continue →
                </button>
              </div>
            </div>
          )}

          {/* Step 3: Learning Goals */}
          {step === 3 && (
            <div className={styles.formStep}>
              <p className={styles.stepDescription}>
                What would you like to achieve with this textbook?
              </p>

              <div className={styles.formGroup}>
                <label>Learning Goals (select all that apply)</label>
                <div className={styles.checkboxGroup}>
                  {LEARNING_GOALS.map(goal => (
                    <label key={goal.id} className={styles.checkboxLabel}>
                      <input
                        type="checkbox"
                        checked={background.learningGoals.includes(goal.id)}
                        onChange={() => handleCheckboxChange('learningGoals', goal.id)}
                      />
                      <span>{goal.label}</span>
                    </label>
                  ))}
                </div>
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="preferredDepth">Preferred Content Depth</label>
                <select
                  id="preferredDepth"
                  name="preferredDepth"
                  value={background.preferredDepth}
                  onChange={handleSelectChange}
                >
                  <option value="beginner">Beginner-friendly (more explanations)</option>
                  <option value="intermediate">Intermediate (balanced)</option>
                  <option value="advanced">Advanced (include math & theory)</option>
                </select>
              </div>

              <div className={styles.buttonGroup}>
                <button type="button" className={styles.secondaryButton} onClick={handleBack}>
                  ← Back
                </button>
                <button
                  type="submit"
                  className={styles.primaryButton}
                  disabled={isLoading}
                >
                  {isLoading ? 'Creating Account...' : 'Create Account'}
                </button>
              </div>
            </div>
          )}
        </form>

        <div className={styles.authFooter}>
          Already have an account? <a href="/auth/signin">Sign in</a>
        </div>
      </div>
    </div>
  );
};

export default SignupForm;
