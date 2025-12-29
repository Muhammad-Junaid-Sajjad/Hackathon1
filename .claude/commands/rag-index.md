# RAG Content Indexer

Index textbook content into Qdrant vector database for RAG chatbot functionality.

## Arguments
- `$ARGUMENTS` - Scope: "all", "M1", "M2/C1", or specific section "M2/C1/S3"

## Instructions

You are the **RAG Content Indexing Specialist**. Your job is to prepare and index textbook content for semantic search.

### Step 1: Discover Content

Based on the scope argument, find all relevant markdown files:

```bash
# For "all"
find docs/ -name "*.md" -type f | head -100

# For module (e.g., "M2")
find docs/M2/ -name "*.md" -type f

# For chapter (e.g., "M2/C1")
find docs/M2/C1/ -name "*.md" -type f
```

### Step 2: Content Extraction Strategy

For each markdown file, extract these chunk types:

#### Type 1: Section Headers
```
Chunk: {title} - {header_text}
Metadata: {section_path, header_level, position}
```

#### Type 2: Code Examples
```
Chunk: {code_block}
Metadata: {language, section_path, context_text}
```

#### Type 3: Concept Definitions
From Key Concepts tables:
```
Chunk: {term}: {definition}. Why It Matters: {importance}
Metadata: {section_path, type: "concept"}
```

#### Type 4: Paragraph Chunks (500-800 tokens)
```
Chunk: {paragraph_text}
Metadata: {section_path, position, surrounding_headers}
```

#### Type 5: Exercises
```
Chunk: Exercise: {title}. Objective: {objective}. {instructions}
Metadata: {section_path, difficulty, type: "exercise"}
```

### Step 3: Generate Chunking Script

Create or update `api/scripts/index_content.py`:

```python
#!/usr/bin/env python3
"""
RAG Content Indexer for Physical AI & Humanoid Robotics Textbook

Usage:
    python index_content.py --scope all
    python index_content.py --scope M2
    python index_content.py --scope M2/C1/S3
"""

import os
import re
import hashlib
import asyncio
from pathlib import Path
from typing import List, Dict, Any
from dataclasses import dataclass

import httpx
from qdrant_client import QdrantClient
from qdrant_client.http import models as rest

# Configuration from environment
QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", "")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "")
COLLECTION_NAME = "robotics_textbook"

@dataclass
class ContentChunk:
    id: str
    text: str
    metadata: Dict[str, Any]

def generate_chunk_id(text: str, metadata: dict) -> str:
    """Generate deterministic chunk ID for deduplication."""
    content = f"{metadata.get('source', '')}-{text[:100]}"
    return hashlib.md5(content.encode()).hexdigest()

def extract_frontmatter(content: str) -> tuple[dict, str]:
    """Extract YAML frontmatter from markdown."""
    if content.startswith('---'):
        parts = content.split('---', 2)
        if len(parts) >= 3:
            import yaml
            try:
                meta = yaml.safe_load(parts[1])
                return meta or {}, parts[2]
            except:
                pass
    return {}, content

def chunk_markdown(filepath: Path) -> List[ContentChunk]:
    """Extract semantic chunks from markdown file."""
    content = filepath.read_text(encoding='utf-8')
    frontmatter, body = extract_frontmatter(content)

    # Derive section path from filepath (e.g., docs/M2/C1/S3.md -> M2-C1-S3)
    rel_path = filepath.relative_to(Path('docs'))
    section_path = str(rel_path.with_suffix('')).replace('/', '-')

    chunks = []

    # Extract title
    title = frontmatter.get('title', section_path)

    # 1. Chunk by headers (## and ###)
    header_pattern = r'^(#{2,3})\s+(.+)$'
    sections = re.split(r'(?=^#{2,3}\s)', body, flags=re.MULTILINE)

    for i, section in enumerate(sections):
        if not section.strip():
            continue

        # Extract header if present
        header_match = re.match(header_pattern, section, re.MULTILINE)
        header = header_match.group(2) if header_match else f"Section {i}"
        header_level = len(header_match.group(1)) if header_match else 2

        # Clean section text
        section_text = re.sub(r'^#{2,3}\s+.+\n', '', section).strip()

        if len(section_text) > 100:  # Only meaningful sections
            chunk = ContentChunk(
                id=generate_chunk_id(section_text, {'source': section_path}),
                text=f"{title} - {header}\n\n{section_text[:2000]}",
                metadata={
                    'source': section_path,
                    'title': title,
                    'header': header,
                    'header_level': header_level,
                    'type': 'section',
                    'filepath': str(filepath),
                }
            )
            chunks.append(chunk)

    # 2. Extract code blocks separately
    code_pattern = r'```(\w+)?\n([\s\S]*?)```'
    for match in re.finditer(code_pattern, body):
        language = match.group(1) or 'text'
        code = match.group(2).strip()

        if len(code) > 50:  # Only substantial code
            # Get surrounding context
            start = max(0, match.start() - 200)
            context = body[start:match.start()].strip()[-200:]

            chunk = ContentChunk(
                id=generate_chunk_id(code, {'source': section_path, 'type': 'code'}),
                text=f"Code example from {title}:\n\nContext: {context}\n\n```{language}\n{code}\n```",
                metadata={
                    'source': section_path,
                    'title': title,
                    'language': language,
                    'type': 'code',
                    'filepath': str(filepath),
                }
            )
            chunks.append(chunk)

    # 3. Extract key concepts from tables
    concept_pattern = r'\|\s*\*\*([^*]+)\*\*\s*\|\s*([^|]+)\|'
    for match in re.finditer(concept_pattern, body):
        term = match.group(1).strip()
        definition = match.group(2).strip()

        chunk = ContentChunk(
            id=generate_chunk_id(term, {'source': section_path, 'type': 'concept'}),
            text=f"Key Concept: {term}\nDefinition: {definition}\nFrom section: {title}",
            metadata={
                'source': section_path,
                'title': title,
                'term': term,
                'type': 'concept',
                'filepath': str(filepath),
            }
        )
        chunks.append(chunk)

    return chunks

async def get_embeddings(texts: List[str]) -> List[List[float]]:
    """Get embeddings from OpenAI."""
    async with httpx.AsyncClient() as client:
        response = await client.post(
            "https://api.openai.com/v1/embeddings",
            headers={"Authorization": f"Bearer {OPENAI_API_KEY}"},
            json={
                "model": "text-embedding-3-small",
                "input": texts
            },
            timeout=60.0
        )
        response.raise_for_status()
        data = response.json()
        return [item['embedding'] for item in data['data']]

async def index_chunks(chunks: List[ContentChunk]):
    """Index chunks to Qdrant."""
    client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY or None)

    # Create collection if not exists
    try:
        client.get_collection(COLLECTION_NAME)
    except:
        client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=rest.VectorParams(
                size=1536,  # text-embedding-3-small
                distance=rest.Distance.COSINE
            )
        )

    # Process in batches
    batch_size = 100
    for i in range(0, len(chunks), batch_size):
        batch = chunks[i:i+batch_size]
        texts = [c.text for c in batch]

        print(f"Embedding batch {i//batch_size + 1}/{(len(chunks)-1)//batch_size + 1}...")
        embeddings = await get_embeddings(texts)

        points = [
            rest.PointStruct(
                id=idx + i,
                vector=emb,
                payload={
                    'text': chunk.text,
                    **chunk.metadata
                }
            )
            for idx, (chunk, emb) in enumerate(zip(batch, embeddings))
        ]

        client.upsert(collection_name=COLLECTION_NAME, points=points)
        print(f"  Indexed {len(points)} chunks")

async def main(scope: str):
    """Main indexing workflow."""
    docs_path = Path('docs')

    if scope == 'all':
        files = list(docs_path.rglob('*.md'))
    elif '/' in scope and scope.count('/') == 2:
        # Specific section like M2/C1/S3
        files = [docs_path / f"{scope}.md"]
    elif '/' in scope:
        # Chapter like M2/C1
        files = list((docs_path / scope).glob('*.md'))
    else:
        # Module like M2
        files = list((docs_path / scope).rglob('*.md'))

    print(f"Found {len(files)} files to index")

    all_chunks = []
    for filepath in files:
        if filepath.exists():
            chunks = chunk_markdown(filepath)
            all_chunks.extend(chunks)
            print(f"  {filepath}: {len(chunks)} chunks")

    print(f"\nTotal chunks: {len(all_chunks)}")
    await index_chunks(all_chunks)
    print("\n✅ Indexing complete!")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--scope', default='all')
    args = parser.parse_args()

    asyncio.run(main(args.scope))
```

### Step 4: Create Collection Schema

Ensure Qdrant collection has proper schema:

```python
# Collection config
{
    "name": "robotics_textbook",
    "vectors": {
        "size": 1536,
        "distance": "Cosine"
    },
    "payload_schema": {
        "source": "keyword",      # M2-C1-S3 format
        "title": "text",
        "header": "text",
        "type": "keyword",        # section, code, concept
        "language": "keyword",    # For code chunks
        "filepath": "keyword"
    }
}
```

### Step 5: Validate Index

After indexing, verify:

```bash
# Test query
curl -X POST "${QDRANT_URL}/collections/robotics_textbook/points/search" \
  -H "Content-Type: application/json" \
  -d '{
    "vector": [... embedding for "ROS 2 node" ...],
    "limit": 5,
    "with_payload": true
  }'
```

### Step 6: Report

```
╔══════════════════════════════════════════════════════════════╗
║              RAG INDEXING REPORT                              ║
╠══════════════════════════════════════════════════════════════╣
║ Scope: [scope argument]                                       ║
║ Files Processed: [count]                                      ║
╠══════════════════════════════════════════════════════════════╣
║ Chunk Type      │ Count   │ Avg Size                          ║
╠═════════════════╪═════════╪═══════════════════════════════════╣
║ Sections        │ [n]     │ [tokens] tokens                   ║
║ Code Blocks     │ [n]     │ [tokens] tokens                   ║
║ Concepts        │ [n]     │ [tokens] tokens                   ║
╠══════════════════════════════════════════════════════════════╣
║ Total Chunks: [total]                                         ║
║ Qdrant Status: [Connected/Error]                             ║
║ Collection: robotics_textbook                                ║
╚══════════════════════════════════════════════════════════════╝

Sample Search Test:
Query: "How to create a ROS 2 node"
Top Result: [source] - [score]
```

## Reusability

- `/rag-index all` - Full textbook indexing
- `/rag-index M2` - Index Module 2 only
- `/rag-index M2/C1` - Index specific chapter
- `/rag-index M2/C1/S3` - Index single section (for updates)

Run after any content changes to keep search current.
