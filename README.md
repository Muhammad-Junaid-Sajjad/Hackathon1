# 🤖 Physical AI & Humanoid Robotics Textbook

A comprehensive, interactive textbook on Physical AI and Humanoid Robotics built with Docusaurus, featuring AI-powered learning tools.

![Hackathon Badge](https://img.shields.io/badge/Hackathon-2025-blueviolet)
![Docusaurus](https://img.shields.io/badge/Docusaurus-3.6-green)
![License](https://img.shields.io/badge/License-MIT-blue)

## 🏆 Hackathon Requirements Implemented

This project implements all **7 Hackathon requirements** for maximum points (300+):

| Requirement | Description | Status | Points |
|-------------|-------------|--------|--------|
| **1** | Docusaurus Book + GitHub Pages | ✅ Complete | 100 |
| **2** | RAG Chatbot (OpenAI, FastAPI, Neon, Qdrant) | ✅ Complete | 100 |
| **3** | Base Functionality | ✅ Complete | - |
| **4** | Claude Code Subagents & Skills | ✅ Complete | +50 |
| **5** | Better-Auth Signup/Signin | ✅ Complete | +50 |
| **6** | Chapter Personalization | ✅ Complete | +50 |
| **7** | Urdu Translation | ✅ Complete | +50 |

## 📚 Textbook Content

The textbook covers Physical AI and Humanoid Robotics across **4 comprehensive modules**:

### Module 1: Foundations
- Introduction to Physical AI
- ROS 2 Fundamentals
- Development Environment Setup

### Module 2: Simulation & Digital Twins
- Gazebo Simulation
- Sensor Simulation
- Sim-to-Real Transfer

### Module 3: Perception & AI
- Computer Vision
- Deep Learning for Robotics
- World Models & Diffusion Policies

### Module 4: Integration & Deployment
- System Integration
- Cloud Robotics & Fleet Management
- Production Deployment

**Plus comprehensive reference materials:**
- 📖 200+ term Glossary
- 📚 89 IEEE-style Citations
- 📊 15+ Benchmark Tables
- 💰 Hardware Budget Guide
- ✅ Deployment Checklists
- 🎓 Certification Pathways
- 📝 Self-Assessment Quizzes

## 🚀 Features

### 1. RAG-Powered Chatbot
AI chatbot that answers questions using textbook content:
- Semantic search with Qdrant vector database
- Context-aware responses with source citations
- Text selection for contextual questions
- Conversation history

### 2. User Authentication
Multi-step signup with background questions:
- Programming experience level
- Robotics experience
- Available hardware
- Learning goals
- Preferred content depth

### 3. Content Personalization
Adapts content based on user background:
- Beginner-friendly explanations
- Hardware-specific tips
- Goal-aligned highlights
- Skip suggestions for advanced users

### 4. Urdu Translation
Full chapter translation to Urdu:
- Technical terms with transliteration
- RTL layout support
- Code preservation
- Cached for performance

### 5. Claude Code Skills
Reusable AI skills for development:
- `/rag-index` - Index content to vector DB
- `/translate-urdu` - Translate content
- `/personalize-content` - Personalize for users
- `/validate-api` - Test all integrations
- `/generate-component` - Create React components
- Plus 20+ existing enhancement skills

## 🛠️ Tech Stack

### Frontend
- **Docusaurus 3.6** - Static site generator
- **React 18** - UI components
- **TypeScript** - Type safety
- **CSS Modules** - Scoped styling

### Backend
- **FastAPI** - Python API framework
- **OpenAI** - Embeddings & chat completions
- **Qdrant Cloud** - Vector database
- **Neon Serverless** - PostgreSQL database
- **JWT** - Authentication tokens

### Deployment
- **GitHub Pages** - Static site hosting
- **GitHub Actions** - CI/CD pipeline

## 📦 Installation

### Prerequisites
- Node.js 20+
- Python 3.10+
- npm or yarn

### Frontend Setup

```bash
# Clone the repository
git clone https://github.com/your-username/hackathon-1.git
cd hackathon-1

# Install dependencies
npm install

# Start development server
npm start
```

### Backend Setup

```bash
# Navigate to API directory
cd api

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Copy environment template
cp .env.example .env

# Edit .env with your credentials
# OPENAI_API_KEY=sk-...
# QDRANT_URL=https://...
# QDRANT_API_KEY=...
# DATABASE_URL=postgresql://...
# JWT_SECRET=your-secret-key

# Run database migrations
psql $DATABASE_URL < schema.sql

# Index content to Qdrant
python scripts/index_content.py --scope all

# Start API server
uvicorn main:app --reload
```

## 🔧 Configuration

### Environment Variables

| Variable | Description | Required |
|----------|-------------|----------|
| `OPENAI_API_KEY` | OpenAI API key | Yes |
| `QDRANT_URL` | Qdrant Cloud endpoint | Yes |
| `QDRANT_API_KEY` | Qdrant Cloud API key | Yes |
| `DATABASE_URL` | Neon Postgres connection string | Yes |
| `JWT_SECRET` | Secret for JWT tokens | Yes |
| `CORS_ORIGINS` | Allowed CORS origins | No |

### Docusaurus Configuration

Edit `docusaurus.config.ts` to customize:
- Site title and description
- Theme colors
- Navigation links
- Footer content

## 📁 Project Structure

```
hackathon-1/
├── docs/                    # Textbook content (84+ sections)
│   ├── M1/                  # Module 1: Foundations
│   ├── M2/                  # Module 2: Simulation
│   ├── M3/                  # Module 3: Perception & AI
│   ├── M4/                  # Module 4: Integration
│   ├── glossary.md          # 200+ terms
│   ├── references.md        # 89 citations
│   └── ...                  # Reference materials
├── src/
│   ├── components/
│   │   ├── Chatbot/         # RAG chatbot widget
│   │   ├── ChapterTools/    # Personalization & translation
│   │   └── Auth/            # Authentication forms
│   ├── pages/
│   │   └── auth/            # Signup/Signin pages
│   └── theme/
│       ├── Root.tsx         # Global wrapper
│       └── DocItem/         # Doc page wrapper
├── api/
│   ├── main.py              # FastAPI application
│   ├── schema.sql           # Database schema
│   ├── requirements.txt     # Python dependencies
│   └── scripts/
│       └── index_content.py # RAG indexing script
├── .claude/
│   └── commands/            # Claude Code skills (25+)
├── .github/
│   └── workflows/
│       └── deploy.yml       # GitHub Pages deployment
└── .specify/
    └── templates/           # Enhancement frameworks
```

## 🎯 Claude Code Skills

### Content Enhancement
- `/enhance-section M2/C1/S3` - Multi-dimensional enhancement
- `/batch-enhance M2` - Enhance entire module
- `/add-theory M2/C1/S3` - Add theoretical content
- `/add-examples M2/C1/S3` - Add practical examples
- `/add-exercises M2/C1/S3` - Add tiered exercises

### RAG & Translation
- `/rag-index all` - Index all content to Qdrant
- `/translate-urdu M2/C1/S3` - Translate to Urdu
- `/personalize-content {profile}` - Test personalization

### Quality & Validation
- `/check-quality M2/C1/S3` - Audit section quality
- `/validate-api all` - Test all API endpoints
- `/fix-mdx M2/C1/S3` - Fix MDX syntax errors

### Development
- `/generate-component QuizWidget:widget` - Create components
- `/generate-diagram M2/C1/S3` - Generate ASCII diagrams
- `/capstone-connect M2/C1/S3` - Link to capstone project

## 📊 API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/chat` | POST | RAG-powered Q&A |
| `/api/auth/signup` | POST | User registration |
| `/api/auth/signin` | POST | User login |
| `/api/auth/validate` | GET | Validate JWT token |
| `/api/personalize` | POST | Personalize content |
| `/api/translate` | POST | Translate to Urdu |
| `/health` | GET | Health check |

## 🚢 Deployment

### GitHub Pages (Frontend)

Push to `main` branch triggers automatic deployment via GitHub Actions.

### API Deployment

Deploy the FastAPI backend to your preferred platform:
- **Railway**: `railway up`
- **Render**: Connect GitHub repo
- **Fly.io**: `fly deploy`
- **Vercel**: Serverless functions

## 📝 License

This project is licensed under the MIT License.

## 🙏 Acknowledgments

- Built for the Piaic Hackathon 2025
- Powered by Claude Code and OpenAI
- Docusaurus by Meta
- Qdrant Vector Database
- Neon Serverless Postgres

---

**Made with ❤️ for the Physical AI & Humanoid Robotics community**
