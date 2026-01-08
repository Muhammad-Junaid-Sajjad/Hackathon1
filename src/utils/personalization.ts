/**
 * User background interface matching the backend API
 */
interface UserBackground {
  programming_experience: string;
  robotics_experience: string;
  hardware_access: string[];
  learning_goals: string[];
  preferred_depth: string;
}

/**
 * Calls the backend API to personalize content based on user background
 */
export const personalizeContent = async (
  chapterContent: string,
  userId: string,
  background: UserBackground
): Promise<string> => {
  try {
    const response = await fetch('/api/personalize', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        chapter_content: chapterContent,
        user_id: userId,
        chapter_id: 'unknown' // In a real implementation, you'd pass the actual chapter ID
      }),
    });

    if (!response.ok) {
      throw new Error(`Personalization API error: ${response.status}`);
    }

    const data = await response.json();
    return data.personalized_content;
  } catch (error) {
    console.error('Error personalizing content:', error);
    // Fallback: return original content if personalization fails
    return chapterContent;
  }
};