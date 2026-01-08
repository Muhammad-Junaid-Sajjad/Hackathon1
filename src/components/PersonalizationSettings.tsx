import React, { useState, useEffect } from 'react';
import { useAuth } from '@site/src/auth/AuthProvider';

interface UserBackground {
  programming_experience: string;
  robotics_experience: string;
  hardware_access: string[];
  learning_goals: string[];
  preferred_depth: string;
}

interface PersonalizationSettingsProps {
  onSave: (background: UserBackground) => void;
}

const PersonalizationSettings: React.FC<PersonalizationSettingsProps> = ({ onSave }) => {
  const { user, isAuthenticated } = useAuth();
  const [background, setBackground] = useState<UserBackground>({
    programming_experience: 'beginner',
    robotics_experience: 'none',
    hardware_access: [],
    learning_goals: [],
    preferred_depth: 'intermediate'
  });
  const [newGoal, setNewGoal] = useState('');
  const [newHardware, setNewHardware] = useState('');

  useEffect(() => {
    if (isAuthenticated && user) {
      fetchUserBackground();
    }
  }, [isAuthenticated, user]);

  const fetchUserBackground = async () => {
    try {
      const response = await fetch(`/api/user/${user?.id}/background`);
      if (response.ok) {
        const data = await response.json();
        setBackground(data);
      }
    } catch (error) {
      console.error('Error fetching user background:', error);
    }
  };

  const handleChange = (field: keyof UserBackground, value: any) => {
    setBackground(prev => ({
      ...prev,
      [field]: value
    }));
  };

  const addGoal = () => {
    if (newGoal.trim() && !background.learning_goals.includes(newGoal.trim())) {
      setBackground(prev => ({
        ...prev,
        learning_goals: [...prev.learning_goals, newGoal.trim()]
      }));
      setNewGoal('');
    }
  };

  const removeGoal = (goal: string) => {
    setBackground(prev => ({
      ...prev,
      learning_goals: prev.learning_goals.filter(g => g !== goal)
    }));
  };

  const addHardware = () => {
    if (newHardware.trim() && !background.hardware_access.includes(newHardware.trim())) {
      setBackground(prev => ({
        ...prev,
        hardware_access: [...prev.hardware_access, newHardware.trim()]
      }));
      setNewHardware('');
    }
  };

  const removeHardware = (item: string) => {
    setBackground(prev => ({
      ...prev,
      hardware_access: prev.hardware_access.filter(h => h !== item)
    }));
  };

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    onSave(background);
  };

  if (!isAuthenticated) {
    return (
      <div className="alert alert--info">
        Please log in to customize your learning experience.
      </div>
    );
  }

  return (
    <form onSubmit={handleSubmit} className="padding--md">
      <h3>Personalize Your Learning Experience</h3>
      
      <div className="margin-bottom--lg">
        <label className="form-label">Programming Experience</label>
        <select 
          value={background.programming_experience}
          onChange={(e) => handleChange('programming_experience', e.target.value)}
          className="form-select"
        >
          <option value="none">None</option>
          <option value="beginner">Beginner</option>
          <option value="intermediate">Intermediate</option>
          <option value="advanced">Advanced</option>
          <option value="expert">Expert</option>
        </select>
      </div>

      <div className="margin-bottom--lg">
        <label className="form-label">Robotics Experience</label>
        <select 
          value={background.robotics_experience}
          onChange={(e) => handleChange('robotics_experience', e.target.value)}
          className="form-select"
        >
          <option value="none">None</option>
          <option value="hobbyist">Hobbyist</option>
          <option value="student">Student</option>
          <option value="professional">Professional</option>
          <option value="researcher">Researcher</option>
        </select>
      </div>

      <div className="margin-bottom--lg">
        <label className="form-label">Preferred Depth</label>
        <select 
          value={background.preferred_depth}
          onChange={(e) => handleChange('preferred_depth', e.target.value)}
          className="form-select"
        >
          <option value="beginner">Beginner</option>
          <option value="intermediate">Intermediate</option>
          <option value="advanced">Advanced</option>
        </select>
      </div>

      <div className="margin-bottom--lg">
        <label className="form-label">Learning Goals</label>
        <div className="input-group">
          <input
            type="text"
            value={newGoal}
            onChange={(e) => setNewGoal(e.target.value)}
            placeholder="Add a learning goal"
            className="form-input"
          />
          <button type="button" onClick={addGoal} className="button button--sm button--primary">
            Add
          </button>
        </div>
        <div className="margin-top--sm">
          {background.learning_goals.map((goal, index) => (
            <span key={index} className="badge badge--secondary margin-right--sm">
              {goal}
              <button 
                type="button" 
                onClick={() => removeGoal(goal)}
                className="margin-left--sm"
              >
                ×
              </button>
            </span>
          ))}
        </div>
      </div>

      <div className="margin-bottom--lg">
        <label className="form-label">Hardware Access</label>
        <div className="input-group">
          <input
            type="text"
            value={newHardware}
            onChange={(e) => setNewHardware(e.target.value)}
            placeholder="Add hardware you have access to"
            className="form-input"
          />
          <button type="button" onClick={addHardware} className="button button--sm button--primary">
            Add
          </button>
        </div>
        <div className="margin-top--sm">
          {background.hardware_access.map((item, index) => (
            <span key={index} className="badge badge--info margin-right--sm">
              {item}
              <button 
                type="button" 
                onClick={() => removeHardware(item)}
                className="margin-left--sm"
              >
                ×
              </button>
            </span>
          ))}
        </div>
      </div>

      <button type="submit" className="button button--primary">
        Save Preferences
      </button>
    </form>
  );
};

export default PersonalizationSettings;