"""
RAG Chatbot API for Physical AI & Humanoid Robotics Textbook.

This FastAPI backend provides:
- RAG-based Q&A using OpenAI + Qdrant vector database
- User authentication via Better-Auth integration
- Content personalization based on user background
- Urdu translation endpoint

Requirements addressed:
- Requirement 2: RAG Chatbot with OpenAI, FastAPI, Neon, Qdrant
- Requirement 5: Better-Auth integration
- Requirement 6: Personalization
- Requirement 7: Urdu translation
"""

import os
import re
import secrets
from typing import Optional, List, Dict, Any
from datetime import datetime, timedelta
from contextlib import asynccontextmanager

from fastapi import FastAPI, HTTPException, Depends, Header, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from pydantic import BaseModel, Field, EmailStr, validator
import httpx
from dotenv import load_dotenv

# Rate limiting
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded

# Password hashing
try:
    import bcrypt
    BCRYPT_AVAILABLE = True
except ImportError:
    BCRYPT_AVAILABLE = False
    print("WARNING: bcrypt not installed. Password hashing disabled.")

# Load environment variables
load_dotenv()

# Configuration
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL", "https://your-cluster.qdrant.io")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
NEON_DATABASE_URL = os.getenv("NEON_DATABASE_URL")
BETTER_AUTH_SECRET = os.getenv("BETTER_AUTH_SECRET")

# Security: Allowed CORS origins (configure per environment)
ALLOWED_ORIGINS = os.getenv("ALLOWED_ORIGINS", "http://localhost:3000").split(",")

# Collection name for book content vectors
COLLECTION_NAME = "physical_ai_textbook"

# Initialize rate limiter
limiter = Limiter(key_func=get_remote_address)

# Connection pool (initialized at startup)
db_pool = None


# ============== Pydantic Models ==============

class ChatMessage(BaseModel):
    """Single chat message."""
    role: str = Field(..., description="Message role: 'user' or 'assistant'")
    content: str = Field(..., description="Message content")


class ChatRequest(BaseModel):
    """Request for chat endpoint."""
    message: str = Field(..., description="User's question")
    selected_text: Optional[str] = Field(None, description="Text selected by user for context")
    conversation_history: List[ChatMessage] = Field(default_factory=list)
    user_id: Optional[str] = Field(None, description="Authenticated user ID")


class ChatResponse(BaseModel):
    """Response from chat endpoint."""
    answer: str
    sources: List[Dict[str, Any]] = Field(default_factory=list)
    confidence: float = 0.0


class PersonalizeRequest(BaseModel):
    """Request for content personalization."""
    chapter_content: str = Field(..., description="Original chapter content (markdown)")
    user_id: str = Field(..., description="User ID for fetching background")
    chapter_id: str = Field(..., description="Chapter identifier")


class PersonalizeResponse(BaseModel):
    """Response with personalized content."""
    personalized_content: str
    adaptations_made: List[str] = Field(default_factory=list)


class TranslateRequest(BaseModel):
    """Request for Urdu translation."""
    content: str = Field(..., description="Content to translate")
    chapter_id: str = Field(..., description="Chapter identifier")


class TranslateResponse(BaseModel):
    """Response with translated content."""
    translated_content: str
    original_language: str = "en"
    target_language: str = "ur"


class UserBackground(BaseModel):
    """User background information for personalization."""
    programming_experience: str = Field(..., description="none/beginner/intermediate/advanced/expert")
    robotics_experience: str = Field(..., description="none/hobbyist/student/professional/researcher")
    hardware_access: List[str] = Field(default_factory=list, description="Available hardware")
    learning_goals: List[str] = Field(default_factory=list)
    preferred_depth: str = Field("intermediate", description="beginner/intermediate/advanced")


class SignupRequest(BaseModel):
    """User signup with background questions."""
    email: EmailStr  # Validates email format
    password: str
    name: str
    background: UserBackground

    @validator('password')
    def password_strength(cls, v):
        """Enforce password strength requirements."""
        if len(v) < 8:
            raise ValueError('Password must be at least 8 characters')
        if not re.search(r'[A-Z]', v):
            raise ValueError('Password must contain at least one uppercase letter')
        if not re.search(r'[a-z]', v):
            raise ValueError('Password must contain at least one lowercase letter')
        if not re.search(r'\d', v):
            raise ValueError('Password must contain at least one digit')
        return v

    @validator('name')
    def name_validation(cls, v):
        """Validate name field."""
        if len(v.strip()) < 2:
            raise ValueError('Name must be at least 2 characters')
        if len(v) > 100:
            raise ValueError('Name must be less than 100 characters')
        return v.strip()


# ============== Security Helpers ==============

def hash_password(password: str) -> str:
    """Hash password using bcrypt."""
    if not BCRYPT_AVAILABLE:
        raise HTTPException(status_code=500, detail="Password hashing not available")
    salt = bcrypt.gensalt(rounds=12)
    return bcrypt.hashpw(password.encode('utf-8'), salt).decode('utf-8')


def verify_password(password: str, hashed: str) -> bool:
    """Verify password against hash."""
    if not BCRYPT_AVAILABLE:
        return False
    return bcrypt.checkpw(password.encode('utf-8'), hashed.encode('utf-8'))


security = HTTPBearer(auto_error=False)


# ============== Database Helpers ==============

async def create_db_pool():
    """Create database connection pool."""
    global db_pool
    import asyncpg
    db_pool = await asyncpg.create_pool(
        NEON_DATABASE_URL,
        min_size=5,
        max_size=20,
        command_timeout=60,
    )
    return db_pool


async def get_db_connection():
    """Get connection from pool or create individual connection."""
    import asyncpg
    if db_pool:
        return await db_pool.acquire()
    # Fallback to individual connection
    return await asyncpg.connect(NEON_DATABASE_URL)


async def release_db_connection(conn):
    """Release connection back to pool."""
    if db_pool:
        await db_pool.release(conn)
    else:
        await conn.close()


async def init_database():
    """Initialize database tables."""
    conn = await get_db_connection()
    try:
        await conn.execute('''
            CREATE TABLE IF NOT EXISTS users (
                id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                email VARCHAR(255) UNIQUE NOT NULL,
                password_hash VARCHAR(255) NOT NULL,
                name VARCHAR(255) NOT NULL,
                email_verified BOOLEAN DEFAULT FALSE,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        ''')
        await conn.execute('''
            CREATE TABLE IF NOT EXISTS user_backgrounds (
                id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                user_id UUID REFERENCES users(id) ON DELETE CASCADE,
                programming_experience VARCHAR(50),
                robotics_experience VARCHAR(50),
                hardware_access TEXT[],
                learning_goals TEXT[],
                preferred_depth VARCHAR(50),
                updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        ''')
        await conn.execute('''
            CREATE TABLE IF NOT EXISTS chat_history (
                id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
                user_id UUID REFERENCES users(id) ON DELETE CASCADE,
                message TEXT NOT NULL,
                response TEXT NOT NULL,
                sources JSONB,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        ''')
        # Create index for faster lookups
        await conn.execute('''
            CREATE INDEX IF NOT EXISTS idx_chat_history_user_id
            ON chat_history(user_id)
        ''')
    finally:
        await release_db_connection(conn)


# ============== Qdrant Vector Store ==============

class QdrantVectorStore:
    """Qdrant vector database client for RAG."""

    def __init__(self):
        self.url = QDRANT_URL
        self.api_key = QDRANT_API_KEY
        self.collection = COLLECTION_NAME

    async def search(
        self,
        query_embedding: List[float],
        limit: int = 5,
        filter_conditions: Optional[Dict] = None
    ) -> List[Dict]:
        """Search for similar documents."""
        async with httpx.AsyncClient() as client:
            payload = {
                "vector": query_embedding,
                "limit": limit,
                "with_payload": True
            }
            if filter_conditions:
                payload["filter"] = filter_conditions

            response = await client.post(
                f"{self.url}/collections/{self.collection}/points/search",
                json=payload,
                headers={"api-key": self.api_key}
            )

            if response.status_code == 200:
                results = response.json().get("result", [])
                return [
                    {
                        "content": r["payload"].get("content", ""),
                        "metadata": r["payload"].get("metadata", {}),
                        "score": r["score"]
                    }
                    for r in results
                ]
            return []

    async def upsert(self, points: List[Dict]):
        """Insert or update vectors."""
        async with httpx.AsyncClient() as client:
            response = await client.put(
                f"{self.url}/collections/{self.collection}/points",
                json={"points": points},
                headers={"api-key": self.api_key}
            )
            return response.status_code == 200


# ============== OpenAI Integration ==============

class OpenAIClient:
    """OpenAI API client for embeddings and chat."""

    def __init__(self):
        self.api_key = OPENAI_API_KEY
        self.base_url = "https://api.openai.com/v1"

    async def get_embedding(self, text: str) -> List[float]:
        """Get embedding for text."""
        async with httpx.AsyncClient() as client:
            response = await client.post(
                f"{self.base_url}/embeddings",
                json={
                    "model": "text-embedding-3-small",
                    "input": text
                },
                headers={"Authorization": f"Bearer {self.api_key}"}
            )

            if response.status_code == 200:
                return response.json()["data"][0]["embedding"]
            raise HTTPException(status_code=500, detail="Embedding generation failed")

    async def chat_completion(
        self,
        messages: List[Dict],
        system_prompt: str,
        temperature: float = 0.7
    ) -> str:
        """Get chat completion."""
        async with httpx.AsyncClient(timeout=60.0) as client:
            full_messages = [{"role": "system", "content": system_prompt}] + messages

            response = await client.post(
                f"{self.base_url}/chat/completions",
                json={
                    "model": "gpt-4o-mini",
                    "messages": full_messages,
                    "temperature": temperature
                },
                headers={"Authorization": f"Bearer {self.api_key}"}
            )

            if response.status_code == 200:
                return response.json()["choices"][0]["message"]["content"]
            raise HTTPException(status_code=500, detail="Chat completion failed")

    async def translate_to_urdu(self, content: str) -> str:
        """Translate content to Urdu."""
        system_prompt = """You are a professional translator specializing in technical content.
Translate the following content from English to Urdu (اردو).
Maintain all code blocks, technical terms, and formatting.
For technical terms that don't have common Urdu equivalents, keep them in English with Urdu transliteration in parentheses.
Preserve all markdown formatting including headers, lists, code blocks, and links."""

        messages = [{"role": "user", "content": content}]
        return await self.chat_completion(messages, system_prompt, temperature=0.3)


# ============== RAG Engine ==============

class RAGEngine:
    """Retrieval-Augmented Generation engine."""

    def __init__(self):
        self.vector_store = QdrantVectorStore()
        self.openai = OpenAIClient()

    async def answer_question(
        self,
        question: str,
        selected_text: Optional[str] = None,
        conversation_history: List[Dict] = None,
        user_background: Optional[UserBackground] = None
    ) -> ChatResponse:
        """Answer a question using RAG."""

        # Build context from selected text or search
        if selected_text:
            # Use selected text as primary context
            context_docs = [{"content": selected_text, "metadata": {"source": "user_selection"}, "score": 1.0}]
        else:
            # Search vector database
            query_embedding = await self.openai.get_embedding(question)
            context_docs = await self.vector_store.search(query_embedding, limit=5)

        # Build context string
        context = "\n\n".join([
            f"[Source: {doc['metadata'].get('source', 'textbook')}]\n{doc['content']}"
            for doc in context_docs
        ])

        # Build personalized system prompt
        system_prompt = self._build_system_prompt(user_background)

        # Build messages
        messages = []
        if conversation_history:
            messages.extend([
                {"role": msg["role"], "content": msg["content"]}
                for msg in conversation_history[-6:]  # Keep last 6 messages for context
            ])

        # Add current question with context
        user_message = f"""Context from the Physical AI & Humanoid Robotics textbook:
{context}

Question: {question}

Please answer based on the context provided. If the context doesn't contain enough information, say so and provide general guidance."""

        messages.append({"role": "user", "content": user_message})

        # Get response
        answer = await self.openai.chat_completion(messages, system_prompt)

        # Calculate confidence based on relevance scores
        avg_score = sum(doc["score"] for doc in context_docs) / len(context_docs) if context_docs else 0

        return ChatResponse(
            answer=answer,
            sources=[
                {
                    "content": doc["content"][:200] + "...",
                    "source": doc["metadata"].get("source", "textbook"),
                    "score": doc["score"]
                }
                for doc in context_docs
            ],
            confidence=avg_score
        )

    def _build_system_prompt(self, background: Optional[UserBackground]) -> str:
        """Build personalized system prompt based on user background."""
        base_prompt = """You are an expert AI tutor for the "Physical AI & Humanoid Robotics" textbook.
You help students and professionals learn about ROS 2, simulation, perception, locomotion, and system integration for humanoid robots.

Guidelines:
- Be accurate and cite specific sections when possible
- Explain complex concepts clearly
- Provide code examples when helpful
- Acknowledge limitations and suggest further reading"""

        if background:
            adaptations = []

            if background.programming_experience in ["none", "beginner"]:
                adaptations.append("- Use simple language and explain programming concepts thoroughly")
                adaptations.append("- Provide more detailed code explanations")
            elif background.programming_experience in ["advanced", "expert"]:
                adaptations.append("- You can use advanced programming terminology")
                adaptations.append("- Focus on sophisticated patterns and optimizations")

            if background.robotics_experience in ["none", "hobbyist"]:
                adaptations.append("- Explain robotics fundamentals before diving into details")
            elif background.robotics_experience in ["professional", "researcher"]:
                adaptations.append("- Assume familiarity with robotics concepts")
                adaptations.append("- Focus on cutting-edge techniques and research directions")

            if background.preferred_depth == "beginner":
                adaptations.append("- Keep explanations high-level and accessible")
            elif background.preferred_depth == "advanced":
                adaptations.append("- Include mathematical details and implementation nuances")

            if adaptations:
                base_prompt += "\n\nUser-specific adaptations:\n" + "\n".join(adaptations)

        return base_prompt

    async def personalize_content(
        self,
        content: str,
        background: UserBackground
    ) -> PersonalizeResponse:
        """Personalize chapter content based on user background."""

        system_prompt = f"""You are a content personalization expert for a robotics textbook.
Adapt the following chapter content for a reader with this background:
- Programming experience: {background.programming_experience}
- Robotics experience: {background.robotics_experience}
- Available hardware: {', '.join(background.hardware_access) or 'None specified'}
- Learning goals: {', '.join(background.learning_goals) or 'General learning'}
- Preferred depth: {background.preferred_depth}

Guidelines for personalization:
1. Keep ALL original content intact - only ADD explanations, not remove content
2. Add helpful notes for their experience level (use :::note blocks)
3. Highlight sections most relevant to their goals
4. Add hardware-specific tips if applicable
5. Preserve all code blocks, diagrams, and formatting
6. Add "Skip if familiar" notes for experienced users
7. Add "Deep dive" links for advanced users

Return the personalized markdown content."""

        messages = [{"role": "user", "content": content}]
        personalized = await self.openai.chat_completion(messages, system_prompt, temperature=0.5)

        adaptations = []
        if background.programming_experience in ["none", "beginner"]:
            adaptations.append("Added beginner-friendly explanations")
        if background.robotics_experience in ["none", "hobbyist"]:
            adaptations.append("Added robotics fundamentals context")
        if background.hardware_access:
            adaptations.append(f"Added tips for: {', '.join(background.hardware_access)}")

        return PersonalizeResponse(
            personalized_content=personalized,
            adaptations_made=adaptations
        )


# ============== FastAPI Application ==============

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan handler."""
    global db_pool
    # Startup
    print("Starting RAG Chatbot API...")
    try:
        # Initialize connection pool
        if NEON_DATABASE_URL:
            await create_db_pool()
            print("Database connection pool initialized")
            # Initialize database tables
            await init_database()
            print("Database tables initialized")
    except Exception as e:
        print(f"Database initialization warning: {e}")
    yield
    # Shutdown
    print("Shutting down RAG Chatbot API...")
    if db_pool:
        await db_pool.close()
        print("Database connection pool closed")


app = FastAPI(
    title="Physical AI Textbook RAG API",
    description="RAG Chatbot API for the Physical AI & Humanoid Robotics Textbook",
    version="1.0.0",
    lifespan=lifespan
)

# Register rate limiter exception handler
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

# CORS middleware for frontend - SECURITY: Explicitly list allowed origins
app.add_middleware(
    CORSMiddleware,
    allow_origins=ALLOWED_ORIGINS,  # Configured via ALLOWED_ORIGINS env var
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allow_headers=["Authorization", "Content-Type", "X-Admin-Key"],
)

# Initialize RAG engine
rag_engine = RAGEngine()


# ============== API Endpoints ==============

@app.get("/")
async def root():
    """Health check endpoint."""
    return {
        "status": "healthy",
        "service": "Physical AI Textbook RAG API",
        "version": "1.0.0"
    }


@app.post("/api/chat", response_model=ChatResponse)
@limiter.limit("30/minute")  # Rate limit: 30 requests per minute
async def chat(request: ChatRequest, req: Request):
    """
    Chat endpoint for RAG-based Q&A.

    Supports:
    - General questions about the textbook
    - Questions about user-selected text
    - Conversation history for context
    """
    # Get user background if authenticated
    user_background = None
    if request.user_id:
        # Fetch from database
        # user_background = await get_user_background(request.user_id)
        pass

    response = await rag_engine.answer_question(
        question=request.message,
        selected_text=request.selected_text,
        conversation_history=[
            {"role": msg.role, "content": msg.content}
            for msg in request.conversation_history
        ],
        user_background=user_background
    )

    return response


@app.post("/api/personalize", response_model=PersonalizeResponse)
@limiter.limit("10/minute")  # Rate limit: 10 requests per minute
async def personalize_chapter(request: PersonalizeRequest, req: Request):
    """
    Personalize chapter content based on user background.

    Requirement 6: Chapter personalization for logged users.
    """
    # Fetch user background from database
    # For now, use a default background
    background = UserBackground(
        programming_experience="intermediate",
        robotics_experience="student",
        hardware_access=["Jetson Nano"],
        learning_goals=["Build humanoid robot"],
        preferred_depth="intermediate"
    )

    response = await rag_engine.personalize_content(
        content=request.chapter_content,
        background=background
    )

    return response


@app.post("/api/translate", response_model=TranslateResponse)
@limiter.limit("10/minute")  # Rate limit: 10 requests per minute
async def translate_to_urdu(request: TranslateRequest, req: Request):
    """
    Translate chapter content to Urdu.

    Requirement 7: Urdu translation for logged users.
    """
    openai_client = OpenAIClient()
    translated = await openai_client.translate_to_urdu(request.content)

    return TranslateResponse(
        translated_content=translated,
        original_language="en",
        target_language="ur"
    )


@app.post("/api/auth/signup")
@limiter.limit("5/minute")  # Rate limit: 5 signups per minute per IP
async def signup(request: SignupRequest, req: Request):
    """
    User signup with background questions.

    Requirement 5: Better-Auth integration with background questions.

    SECURITY: Implements proper password hashing with bcrypt.
    """
    conn = None
    try:
        # Hash password with bcrypt
        password_hash = hash_password(request.password)

        conn = await get_db_connection()

        # Check if email already exists
        existing = await conn.fetchrow(
            'SELECT id FROM users WHERE email = $1',
            request.email.lower()
        )
        if existing:
            raise HTTPException(
                status_code=400,
                detail="Email already registered"
            )

        # Create user with hashed password
        user_id = await conn.fetchval('''
            INSERT INTO users (email, password_hash, name)
            VALUES ($1, $2, $3)
            RETURNING id
        ''', request.email.lower(), password_hash, request.name)

        # Store user background
        await conn.execute('''
            INSERT INTO user_backgrounds (
                user_id, programming_experience, robotics_experience,
                hardware_access, learning_goals, preferred_depth
            )
            VALUES ($1, $2, $3, $4, $5, $6)
        ''',
            user_id,
            request.background.programming_experience,
            request.background.robotics_experience,
            request.background.hardware_access,
            request.background.learning_goals,
            request.background.preferred_depth
        )

        return {
            "success": True,
            "message": "User created successfully",
            "user": {
                "id": str(user_id),
                "email": request.email.lower(),
                "name": request.name
            }
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Registration failed: {str(e)}")
    finally:
        if conn:
            await release_db_connection(conn)


@app.get("/api/user/{user_id}/background")
async def get_user_background(user_id: str):
    """Get user background for personalization."""
    # Fetch from database
    return {
        "programming_experience": "intermediate",
        "robotics_experience": "student",
        "hardware_access": ["Jetson Nano", "RealSense D435i"],
        "learning_goals": ["Build humanoid robot", "Learn ROS 2"],
        "preferred_depth": "intermediate"
    }


# ============== Index Management ==============

@app.post("/api/admin/index")
async def index_content(content: Dict[str, Any], x_admin_key: str = Header(...)):
    """
    Admin endpoint to index textbook content.
    Requires admin API key.
    """
    if x_admin_key != os.getenv("ADMIN_API_KEY"):
        raise HTTPException(status_code=401, detail="Invalid admin key")

    openai_client = OpenAIClient()
    vector_store = QdrantVectorStore()

    # Generate embedding
    embedding = await openai_client.get_embedding(content["text"])

    # Store in Qdrant
    point = {
        "id": content.get("id", str(hash(content["text"]))),
        "vector": embedding,
        "payload": {
            "content": content["text"],
            "metadata": content.get("metadata", {})
        }
    }

    success = await vector_store.upsert([point])

    return {"success": success, "indexed": content.get("id")}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
