# API Integration Validator

Validate all API endpoints, database connections, and service integrations for the Hackathon requirements.

## Arguments
- `$ARGUMENTS` - "all", "auth", "chatbot", "personalize", "translate", or specific endpoint

## Instructions

You are the **API Integration Validator**. Your job is to ensure all backend services are properly configured and working before deployment.

### Hackathon Requirements Mapping

| Requirement | Endpoint | Dependencies |
|-------------|----------|--------------|
| RAG Chatbot | `/api/chat` | OpenAI, Qdrant |
| Auth | `/api/auth/signup`, `/api/auth/signin` | Neon Postgres |
| Personalize | `/api/personalize` | OpenAI, Neon Postgres |
| Translate | `/api/translate` | OpenAI |

### Step 1: Environment Validation

Check all required environment variables:

```bash
# Create validation script
cat > api/scripts/validate_env.py << 'EOF'
#!/usr/bin/env python3
"""Validate environment configuration."""

import os
import sys

REQUIRED_VARS = {
    # OpenAI
    'OPENAI_API_KEY': {
        'description': 'OpenAI API key for embeddings and chat',
        'prefix': 'sk-',
        'required': True
    },
    # Qdrant
    'QDRANT_URL': {
        'description': 'Qdrant Cloud URL',
        'example': 'https://xxx.qdrant.io:6333',
        'required': True
    },
    'QDRANT_API_KEY': {
        'description': 'Qdrant Cloud API key',
        'required': True
    },
    # Neon Postgres
    'DATABASE_URL': {
        'description': 'Neon Postgres connection string',
        'prefix': 'postgresql://',
        'required': True
    },
    # Auth
    'JWT_SECRET': {
        'description': 'Secret for JWT token signing',
        'min_length': 32,
        'required': True
    },
    # Optional
    'CORS_ORIGINS': {
        'description': 'Allowed CORS origins',
        'default': '*',
        'required': False
    }
}

def validate():
    errors = []
    warnings = []

    for var, config in REQUIRED_VARS.items():
        value = os.getenv(var)

        if not value:
            if config['required']:
                errors.append(f"❌ {var}: Missing (required)")
            else:
                warnings.append(f"⚠️ {var}: Not set, using default '{config.get('default', '')}'")
            continue

        # Prefix check
        if 'prefix' in config and not value.startswith(config['prefix']):
            errors.append(f"❌ {var}: Should start with '{config['prefix']}'")
            continue

        # Length check
        if 'min_length' in config and len(value) < config['min_length']:
            errors.append(f"❌ {var}: Too short (min {config['min_length']} chars)")
            continue

        print(f"✅ {var}: Configured ({config['description']})")

    if warnings:
        print("\nWarnings:")
        for w in warnings:
            print(f"  {w}")

    if errors:
        print("\nErrors:")
        for e in errors:
            print(f"  {e}")
        sys.exit(1)

    print("\n✅ All required environment variables configured!")

if __name__ == "__main__":
    validate()
EOF
python3 api/scripts/validate_env.py
```

### Step 2: Service Connectivity Tests

```python
#!/usr/bin/env python3
"""Test connectivity to all external services."""

import asyncio
import os
import httpx
from qdrant_client import QdrantClient
import asyncpg

async def test_openai():
    """Test OpenAI API connectivity."""
    print("Testing OpenAI API...")
    async with httpx.AsyncClient() as client:
        response = await client.get(
            "https://api.openai.com/v1/models",
            headers={"Authorization": f"Bearer {os.getenv('OPENAI_API_KEY')}"},
            timeout=10.0
        )
        if response.status_code == 200:
            print("  ✅ OpenAI API: Connected")
            return True
        else:
            print(f"  ❌ OpenAI API: {response.status_code}")
            return False

def test_qdrant():
    """Test Qdrant connectivity."""
    print("Testing Qdrant...")
    try:
        client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY")
        )
        collections = client.get_collections()
        print(f"  ✅ Qdrant: Connected ({len(collections.collections)} collections)")
        return True
    except Exception as e:
        print(f"  ❌ Qdrant: {e}")
        return False

async def test_neon():
    """Test Neon Postgres connectivity."""
    print("Testing Neon Postgres...")
    try:
        conn = await asyncpg.connect(os.getenv("DATABASE_URL"))
        version = await conn.fetchval("SELECT version()")
        await conn.close()
        print(f"  ✅ Neon Postgres: Connected")
        return True
    except Exception as e:
        print(f"  ❌ Neon Postgres: {e}")
        return False

async def main():
    results = {
        'openai': await test_openai(),
        'qdrant': test_qdrant(),
        'neon': await test_neon()
    }

    print("\n" + "="*50)
    all_passed = all(results.values())
    if all_passed:
        print("✅ All services connected successfully!")
    else:
        failed = [k for k, v in results.items() if not v]
        print(f"❌ Failed services: {', '.join(failed)}")

    return all_passed

if __name__ == "__main__":
    asyncio.run(main())
```

### Step 3: Endpoint Tests

Test each API endpoint:

```python
#!/usr/bin/env python3
"""Test all API endpoints."""

import asyncio
import httpx

BASE_URL = "http://localhost:8000"

async def test_health():
    """Test health endpoint."""
    async with httpx.AsyncClient() as client:
        response = await client.get(f"{BASE_URL}/health")
        assert response.status_code == 200
        print("✅ GET /health")

async def test_chat():
    """Test RAG chat endpoint."""
    async with httpx.AsyncClient() as client:
        response = await client.post(
            f"{BASE_URL}/api/chat",
            json={
                "message": "What is ROS 2?",
                "conversation_id": "test-123"
            },
            timeout=30.0
        )
        assert response.status_code == 200
        data = response.json()
        assert "response" in data
        assert "sources" in data
        print("✅ POST /api/chat")
        print(f"   Response preview: {data['response'][:100]}...")

async def test_auth_signup():
    """Test signup endpoint."""
    async with httpx.AsyncClient() as client:
        response = await client.post(
            f"{BASE_URL}/api/auth/signup",
            json={
                "email": f"test_{asyncio.get_event_loop().time()}@example.com",
                "password": "TestPass123!",
                "name": "Test User",
                "background": {
                    "programming_experience": "intermediate",
                    "robotics_experience": "student",
                    "hardware_access": ["simulation-only"],
                    "learning_goals": ["ros2"],
                    "preferred_depth": "intermediate"
                }
            }
        )
        # 201 Created or 400 if email exists (OK for repeat tests)
        assert response.status_code in [201, 400]
        print("✅ POST /api/auth/signup")

async def test_personalize():
    """Test personalization endpoint."""
    async with httpx.AsyncClient() as client:
        response = await client.post(
            f"{BASE_URL}/api/personalize",
            json={
                "chapter_content": "# Introduction to ROS 2\n\nROS 2 is a robotics middleware...",
                "user_id": "test-user-id",
                "chapter_id": "M1-C1-S1"
            },
            timeout=60.0
        )
        assert response.status_code == 200
        data = response.json()
        assert "personalized_content" in data
        print("✅ POST /api/personalize")

async def test_translate():
    """Test translation endpoint."""
    async with httpx.AsyncClient() as client:
        response = await client.post(
            f"{BASE_URL}/api/translate",
            json={
                "content": "# Introduction\n\nWelcome to the robotics textbook.",
                "chapter_id": "M1-C1-S1"
            },
            timeout=60.0
        )
        assert response.status_code == 200
        data = response.json()
        assert "translated_content" in data
        print("✅ POST /api/translate")
        print(f"   Translation preview: {data['translated_content'][:100]}...")

async def main():
    print("="*50)
    print("API ENDPOINT VALIDATION")
    print("="*50 + "\n")

    tests = [
        ("Health Check", test_health),
        ("RAG Chat", test_chat),
        ("Auth Signup", test_auth_signup),
        ("Personalization", test_personalize),
        ("Translation", test_translate),
    ]

    results = {}
    for name, test_fn in tests:
        print(f"\nTesting {name}...")
        try:
            await test_fn()
            results[name] = True
        except Exception as e:
            print(f"❌ {name}: {e}")
            results[name] = False

    print("\n" + "="*50)
    print("SUMMARY")
    print("="*50)
    for name, passed in results.items():
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"  {name}: {status}")

    all_passed = all(results.values())
    if all_passed:
        print("\n✅ All endpoints working!")
    else:
        print(f"\n❌ {sum(1 for v in results.values() if not v)} endpoint(s) failed")

if __name__ == "__main__":
    asyncio.run(main())
```

### Step 4: Database Schema Validation

```sql
-- Verify database tables exist
SELECT table_name FROM information_schema.tables
WHERE table_schema = 'public';

-- Expected tables:
-- users, user_backgrounds, conversations, messages, translations_cache

-- Check users table structure
SELECT column_name, data_type, is_nullable
FROM information_schema.columns
WHERE table_name = 'users';
```

### Step 5: Qdrant Collection Validation

```python
from qdrant_client import QdrantClient

client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

# Check collection exists
collection = client.get_collection("robotics_textbook")
print(f"Collection: {collection.points_count} points indexed")

# Test search
results = client.search(
    collection_name="robotics_textbook",
    query_vector=[0.1] * 1536,  # Dummy vector
    limit=1
)
print(f"Search test: {len(results)} results")
```

### Step 6: Report

Generate validation report:

```
╔══════════════════════════════════════════════════════════════╗
║              API VALIDATION REPORT                            ║
╠══════════════════════════════════════════════════════════════╣
║ Environment Variables:                                        ║
║ ├─ OPENAI_API_KEY: ✅ Configured                              ║
║ ├─ QDRANT_URL: ✅ Configured                                  ║
║ ├─ QDRANT_API_KEY: ✅ Configured                              ║
║ ├─ DATABASE_URL: ✅ Configured                                ║
║ └─ JWT_SECRET: ✅ Configured                                  ║
╠══════════════════════════════════════════════════════════════╣
║ Service Connectivity:                                         ║
║ ├─ OpenAI API: ✅ Connected (gpt-4o, text-embedding-3-small) ║
║ ├─ Qdrant Cloud: ✅ Connected (1 collection)                  ║
║ └─ Neon Postgres: ✅ Connected (PostgreSQL 15.x)             ║
╠══════════════════════════════════════════════════════════════╣
║ API Endpoints:                                                ║
║ ├─ GET  /health: ✅ 200 OK                                    ║
║ ├─ POST /api/chat: ✅ 200 OK (~1.2s)                          ║
║ ├─ POST /api/auth/signup: ✅ 201 Created                      ║
║ ├─ POST /api/auth/signin: ✅ 200 OK                           ║
║ ├─ POST /api/personalize: ✅ 200 OK (~3.5s)                   ║
║ └─ POST /api/translate: ✅ 200 OK (~4.2s)                     ║
╠══════════════════════════════════════════════════════════════╣
║ Database Schema:                                              ║
║ ├─ users: ✅ Exists (5 columns)                               ║
║ ├─ user_backgrounds: ✅ Exists (7 columns)                    ║
║ ├─ conversations: ✅ Exists (4 columns)                       ║
║ └─ messages: ✅ Exists (5 columns)                            ║
╠══════════════════════════════════════════════════════════════╣
║ Qdrant Index:                                                 ║
║ ├─ Collection: robotics_textbook                              ║
║ ├─ Points: [count] vectors indexed                            ║
║ ├─ Vector Size: 1536                                          ║
║ └─ Search Test: ✅ Working                                    ║
╠══════════════════════════════════════════════════════════════╣
║ Overall Status: ✅ READY FOR DEPLOYMENT                       ║
╚══════════════════════════════════════════════════════════════╝

Hackathon Requirements Coverage:
✅ Requirement 2: RAG Chatbot - All components working
✅ Requirement 5: Auth - Signup/Signin functional
✅ Requirement 6: Personalization - API ready
✅ Requirement 7: Translation - API ready
```

## Reusability

- `/validate-api all` - Full validation suite
- `/validate-api auth` - Test only auth endpoints
- `/validate-api chatbot` - Test RAG chat endpoint
- `/validate-api personalize` - Test personalization
- `/validate-api translate` - Test translation

Run before deployment to ensure all services are ready.
