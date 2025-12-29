-- Database Schema for Physical AI & Humanoid Robotics Textbook
-- Platform: Neon Serverless Postgres
--
-- This schema supports:
-- - Requirement 5: User authentication with background questions
-- - Requirement 6: Personalization based on user profile
-- - Requirement 7: Translation caching

-- Enable UUID extension
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

-- ============================================
-- USERS TABLE
-- Core user information for authentication
-- ============================================
CREATE TABLE IF NOT EXISTS users (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    name VARCHAR(255) NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    last_login TIMESTAMP WITH TIME ZONE,
    is_active BOOLEAN DEFAULT TRUE
);

-- Index for email lookups during login
CREATE INDEX IF NOT EXISTS idx_users_email ON users(email);

-- ============================================
-- USER BACKGROUNDS TABLE
-- Stores user responses to background questions
-- Used for personalization (Requirement 6)
-- ============================================
CREATE TABLE IF NOT EXISTS user_backgrounds (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    user_id UUID UNIQUE REFERENCES users(id) ON DELETE CASCADE,
    programming_experience VARCHAR(50) NOT NULL,  -- none, beginner, intermediate, advanced, expert
    robotics_experience VARCHAR(50) NOT NULL,     -- none, hobbyist, student, professional, researcher
    hardware_access TEXT[],                        -- Array of hardware IDs
    learning_goals TEXT[],                         -- Array of goal IDs
    preferred_depth VARCHAR(50) DEFAULT 'intermediate', -- beginner, intermediate, advanced
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);

-- ============================================
-- CONVERSATIONS TABLE
-- Stores chat conversation sessions
-- Used for RAG chatbot (Requirement 2)
-- ============================================
CREATE TABLE IF NOT EXISTS conversations (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    user_id UUID REFERENCES users(id) ON DELETE SET NULL, -- NULL for anonymous users
    title VARCHAR(255),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);

-- Index for user conversations
CREATE INDEX IF NOT EXISTS idx_conversations_user_id ON conversations(user_id);

-- ============================================
-- MESSAGES TABLE
-- Stores individual chat messages
-- ============================================
CREATE TABLE IF NOT EXISTS messages (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    conversation_id UUID REFERENCES conversations(id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL,  -- 'user' or 'assistant'
    content TEXT NOT NULL,
    sources JSONB,              -- Array of source references from RAG
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);

-- Index for conversation messages
CREATE INDEX IF NOT EXISTS idx_messages_conversation_id ON messages(conversation_id);

-- ============================================
-- TRANSLATIONS CACHE TABLE
-- Caches Urdu translations to reduce API calls
-- Used for Requirement 7
-- ============================================
CREATE TABLE IF NOT EXISTS translations_cache (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    chapter_id VARCHAR(100) NOT NULL,
    content_hash VARCHAR(64) NOT NULL,  -- SHA-256 of original content
    original_content TEXT NOT NULL,
    translated_content TEXT NOT NULL,
    language VARCHAR(10) DEFAULT 'ur',  -- Urdu
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    expires_at TIMESTAMP WITH TIME ZONE DEFAULT (CURRENT_TIMESTAMP + INTERVAL '30 days')
);

-- Index for fast cache lookups
CREATE UNIQUE INDEX IF NOT EXISTS idx_translations_chapter_hash ON translations_cache(chapter_id, content_hash);

-- ============================================
-- PERSONALIZATION CACHE TABLE
-- Caches personalized content by user profile
-- ============================================
CREATE TABLE IF NOT EXISTS personalization_cache (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    chapter_id VARCHAR(100) NOT NULL,
    user_profile_hash VARCHAR(64) NOT NULL,  -- Hash of user's background profile
    content_hash VARCHAR(64) NOT NULL,       -- Hash of original content
    personalized_content TEXT NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    expires_at TIMESTAMP WITH TIME ZONE DEFAULT (CURRENT_TIMESTAMP + INTERVAL '7 days')
);

-- Index for cache lookups
CREATE UNIQUE INDEX IF NOT EXISTS idx_personalization_lookup ON personalization_cache(chapter_id, user_profile_hash, content_hash);

-- ============================================
-- USER PROGRESS TABLE
-- Tracks user progress through textbook
-- Future feature for learning analytics
-- ============================================
CREATE TABLE IF NOT EXISTS user_progress (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,
    chapter_id VARCHAR(100) NOT NULL,
    completed BOOLEAN DEFAULT FALSE,
    completion_date TIMESTAMP WITH TIME ZONE,
    time_spent_seconds INTEGER DEFAULT 0,
    notes TEXT,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(user_id, chapter_id)
);

-- Index for user progress queries
CREATE INDEX IF NOT EXISTS idx_user_progress_user_id ON user_progress(user_id);

-- ============================================
-- REFRESH TOKENS TABLE
-- For JWT refresh token rotation
-- ============================================
CREATE TABLE IF NOT EXISTS refresh_tokens (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,
    token_hash VARCHAR(64) NOT NULL,
    expires_at TIMESTAMP WITH TIME ZONE NOT NULL,
    revoked BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);

-- Index for token validation
CREATE INDEX IF NOT EXISTS idx_refresh_tokens_user_id ON refresh_tokens(user_id);
CREATE INDEX IF NOT EXISTS idx_refresh_tokens_hash ON refresh_tokens(token_hash);

-- ============================================
-- FUNCTIONS
-- ============================================

-- Function to update updated_at timestamp
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = CURRENT_TIMESTAMP;
    RETURN NEW;
END;
$$ language 'plpgsql';

-- Apply trigger to tables with updated_at
CREATE TRIGGER update_users_updated_at BEFORE UPDATE ON users
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_user_backgrounds_updated_at BEFORE UPDATE ON user_backgrounds
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_conversations_updated_at BEFORE UPDATE ON conversations
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_user_progress_updated_at BEFORE UPDATE ON user_progress
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

-- ============================================
-- CLEANUP FUNCTION
-- Removes expired cache entries
-- ============================================
CREATE OR REPLACE FUNCTION cleanup_expired_cache()
RETURNS void AS $$
BEGIN
    DELETE FROM translations_cache WHERE expires_at < CURRENT_TIMESTAMP;
    DELETE FROM personalization_cache WHERE expires_at < CURRENT_TIMESTAMP;
    DELETE FROM refresh_tokens WHERE expires_at < CURRENT_TIMESTAMP OR revoked = TRUE;
END;
$$ language 'plpgsql';

-- ============================================
-- SAMPLE DATA (Development Only)
-- ============================================
-- Uncomment for development testing

-- INSERT INTO users (email, password_hash, name) VALUES
-- ('test@example.com', '$2b$12$...', 'Test User');

-- INSERT INTO user_backgrounds (user_id, programming_experience, robotics_experience, hardware_access, learning_goals, preferred_depth)
-- SELECT id, 'intermediate', 'student', ARRAY['simulation-only', 'jetson-nano'], ARRAY['ros2', 'perception'], 'intermediate'
-- FROM users WHERE email = 'test@example.com';

COMMENT ON TABLE users IS 'Core user accounts for authentication';
COMMENT ON TABLE user_backgrounds IS 'User background info for content personalization';
COMMENT ON TABLE conversations IS 'RAG chatbot conversation sessions';
COMMENT ON TABLE messages IS 'Individual messages in conversations';
COMMENT ON TABLE translations_cache IS 'Cache for Urdu translations';
COMMENT ON TABLE personalization_cache IS 'Cache for personalized content';
COMMENT ON TABLE user_progress IS 'Tracks user progress through chapters';
COMMENT ON TABLE refresh_tokens IS 'JWT refresh tokens for session management';
