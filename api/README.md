# 🧠 The Physical AI Intelligence Layer (Backend)

This is the power plant of the project. It provides the **Retrievable Memory** (RAG) and **User Adaptation** (Better-Auth) logic that makes the textbook interactive.

## 🚀 Key Technologies
*   **FastAPI**: High-performance Python framework for low-latency AI inference.
*   **Qdrant Cloud**: Vector database for lightning-fast semantic search across 2025 robotics content.
*   **Neon Postgres**: Serverless relational database for user profile and session storage.
*   **OpenAI SDK**: Provides embeddings (text-embedding-3-small) and RAG reasoning (GPT-4o).

## 📁 Intelligence Architecture
1.  **`main.py`**: The API Gateway (Endpoints for `/chat`, `/personalize`, `/translate`).
2.  **`scripts/index_content.py`**: The "Digester" that chunk-indexes the entire textbook into the vector cloud.
3.  **`schema.sql`**: The blueprint for the unified user and hardware-background database.

---
🤖 *Where the digital mind interacts with the physical curriculum.*
