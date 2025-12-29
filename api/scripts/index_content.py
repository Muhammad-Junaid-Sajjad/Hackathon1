#!/usr/bin/env python3
"""
RAG Content Indexer for Physical AI & Humanoid Robotics Textbook

Indexes textbook content into Qdrant vector database for semantic search.

Usage:
    python index_content.py --scope all
    python index_content.py --scope M2
    python index_content.py --scope M2/C1/S3
    python index_content.py --scope all --dry-run

Requirements:
    - OPENAI_API_KEY: OpenAI API key for embeddings
    - QDRANT_URL: Qdrant Cloud URL
    - QDRANT_API_KEY: Qdrant Cloud API key
"""

import os
import re
import hashlib
import asyncio
import argparse
from pathlib import Path
from typing import List, Dict, Any, Tuple
from dataclasses import dataclass

import yaml
import httpx
from qdrant_client import QdrantClient
from qdrant_client.http import models as rest

# Configuration from environment
QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", "")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "")
COLLECTION_NAME = "robotics_textbook"
EMBEDDING_MODEL = "text-embedding-3-small"
EMBEDDING_DIMENSION = 1536


@dataclass
class ContentChunk:
    """Represents a chunk of content to be indexed."""
    id: str
    text: str
    metadata: Dict[str, Any]


def generate_chunk_id(text: str, metadata: dict) -> str:
    """Generate deterministic chunk ID for deduplication."""
    content = f"{metadata.get('source', '')}-{metadata.get('type', '')}-{text[:100]}"
    return hashlib.md5(content.encode()).hexdigest()


def extract_frontmatter(content: str) -> Tuple[dict, str]:
    """Extract YAML frontmatter from markdown."""
    if content.startswith('---'):
        parts = content.split('---', 2)
        if len(parts) >= 3:
            try:
                meta = yaml.safe_load(parts[1])
                return meta or {}, parts[2]
            except yaml.YAMLError:
                pass
    return {}, content


def clean_text(text: str) -> str:
    """Clean markdown text for embedding."""
    # Remove HTML tags
    text = re.sub(r'<[^>]+>', '', text)
    # Remove multiple newlines
    text = re.sub(r'\n{3,}', '\n\n', text)
    # Remove leading/trailing whitespace
    text = text.strip()
    return text


def chunk_markdown(filepath: Path, docs_root: Path) -> List[ContentChunk]:
    """Extract semantic chunks from a markdown file."""
    content = filepath.read_text(encoding='utf-8')
    frontmatter, body = extract_frontmatter(content)

    # Derive section path from filepath (e.g., docs/M2/C1/S3.md -> M2-C1-S3)
    try:
        rel_path = filepath.relative_to(docs_root)
        section_path = str(rel_path.with_suffix('')).replace('/', '-').replace('\\', '-')
    except ValueError:
        section_path = filepath.stem

    chunks = []
    title = frontmatter.get('title', section_path)
    sidebar_label = frontmatter.get('sidebar_label', title)

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
        section_text = clean_text(section_text)

        # Only index meaningful sections (> 100 chars)
        if len(section_text) > 100:
            # Truncate to reasonable size for embedding
            truncated_text = section_text[:2000] if len(section_text) > 2000 else section_text

            chunk = ContentChunk(
                id=generate_chunk_id(truncated_text, {'source': section_path, 'type': 'section'}),
                text=f"{title} - {header}\n\n{truncated_text}",
                metadata={
                    'source': section_path,
                    'title': title,
                    'sidebar_label': sidebar_label,
                    'header': header,
                    'header_level': header_level,
                    'type': 'section',
                    'filepath': str(filepath),
                }
            )
            chunks.append(chunk)

    # 2. Extract code blocks with context
    code_pattern = r'```(\w+)?\n([\s\S]*?)```'
    for match in re.finditer(code_pattern, body):
        language = match.group(1) or 'text'
        code = match.group(2).strip()

        # Only index substantial code (> 50 chars)
        if len(code) > 50:
            # Get surrounding context (before the code block)
            start = max(0, match.start() - 300)
            context = body[start:match.start()]
            # Get last paragraph before code
            context_lines = context.strip().split('\n')
            context_text = ' '.join(context_lines[-3:]).strip()[:200]

            chunk = ContentChunk(
                id=generate_chunk_id(code, {'source': section_path, 'type': 'code'}),
                text=f"Code example from {title}:\n\nContext: {context_text}\n\n```{language}\n{code[:1000]}\n```",
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
    # Match | **term** | definition | patterns
    concept_pattern = r'\|\s*\*\*([^*|]+)\*\*\s*\|\s*([^|]+)\|'
    for match in re.finditer(concept_pattern, body):
        term = match.group(1).strip()
        definition = match.group(2).strip()

        if term and definition and len(definition) > 20:
            chunk = ContentChunk(
                id=generate_chunk_id(term, {'source': section_path, 'type': 'concept'}),
                text=f"Key Concept: {term}\n\nDefinition: {definition}\n\nFrom section: {title}",
                metadata={
                    'source': section_path,
                    'title': title,
                    'term': term,
                    'type': 'concept',
                    'filepath': str(filepath),
                }
            )
            chunks.append(chunk)

    # 4. Extract callout boxes (:::note, :::tip, etc.)
    callout_pattern = r':::(\w+)(?:\s+([^\n]+))?\n([\s\S]*?):::'
    for match in re.finditer(callout_pattern, body):
        callout_type = match.group(1)
        callout_title = match.group(2) or callout_type.title()
        callout_content = clean_text(match.group(3))

        if len(callout_content) > 50:
            chunk = ContentChunk(
                id=generate_chunk_id(callout_content, {'source': section_path, 'type': 'callout'}),
                text=f"{callout_type.upper()}: {callout_title}\n\n{callout_content[:1000]}\n\nFrom: {title}",
                metadata={
                    'source': section_path,
                    'title': title,
                    'callout_type': callout_type,
                    'type': 'callout',
                    'filepath': str(filepath),
                }
            )
            chunks.append(chunk)

    return chunks


async def get_embeddings(texts: List[str]) -> List[List[float]]:
    """Get embeddings from OpenAI API."""
    if not OPENAI_API_KEY:
        raise ValueError("OPENAI_API_KEY environment variable not set")

    async with httpx.AsyncClient() as client:
        response = await client.post(
            "https://api.openai.com/v1/embeddings",
            headers={
                "Authorization": f"Bearer {OPENAI_API_KEY}",
                "Content-Type": "application/json"
            },
            json={
                "model": EMBEDDING_MODEL,
                "input": texts
            },
            timeout=60.0
        )
        response.raise_for_status()
        data = response.json()
        return [item['embedding'] for item in data['data']]


def create_collection(client: QdrantClient):
    """Create Qdrant collection if it doesn't exist."""
    try:
        client.get_collection(COLLECTION_NAME)
        print(f"Collection '{COLLECTION_NAME}' already exists")
    except Exception:
        client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=rest.VectorParams(
                size=EMBEDDING_DIMENSION,
                distance=rest.Distance.COSINE
            )
        )
        print(f"Created collection '{COLLECTION_NAME}'")


async def index_chunks(chunks: List[ContentChunk], dry_run: bool = False):
    """Index chunks to Qdrant."""
    if dry_run:
        print(f"\n[DRY RUN] Would index {len(chunks)} chunks")
        for chunk in chunks[:5]:
            print(f"  - {chunk.metadata['source']}: {chunk.metadata['type']} ({len(chunk.text)} chars)")
        if len(chunks) > 5:
            print(f"  ... and {len(chunks) - 5} more")
        return

    if not QDRANT_URL:
        raise ValueError("QDRANT_URL environment variable not set")

    client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY or None
    )

    # Create collection if needed
    create_collection(client)

    # Process in batches
    batch_size = 50  # Smaller batches to avoid rate limits
    total_indexed = 0

    for i in range(0, len(chunks), batch_size):
        batch = chunks[i:i + batch_size]
        texts = [c.text for c in batch]

        print(f"Processing batch {i // batch_size + 1}/{(len(chunks) - 1) // batch_size + 1} ({len(batch)} chunks)...")

        # Get embeddings
        try:
            embeddings = await get_embeddings(texts)
        except Exception as e:
            print(f"  Error getting embeddings: {e}")
            continue

        # Create points
        points = [
            rest.PointStruct(
                id=idx + i,  # Unique ID across batches
                vector=emb,
                payload={
                    'text': chunk.text,
                    **chunk.metadata
                }
            )
            for idx, (chunk, emb) in enumerate(zip(batch, embeddings))
        ]

        # Upsert to Qdrant
        try:
            client.upsert(collection_name=COLLECTION_NAME, points=points)
            total_indexed += len(points)
            print(f"  ✓ Indexed {len(points)} chunks (total: {total_indexed})")
        except Exception as e:
            print(f"  Error upserting: {e}")

        # Small delay to avoid rate limits
        await asyncio.sleep(0.5)

    print(f"\n✅ Indexing complete! Total chunks indexed: {total_indexed}")


def find_files(scope: str, docs_root: Path) -> List[Path]:
    """Find markdown files based on scope."""
    if scope == 'all':
        return list(docs_root.rglob('*.md'))
    elif '/' in scope and scope.count('/') == 2:
        # Specific section like M2/C1/S3
        filepath = docs_root / f"{scope}.md"
        return [filepath] if filepath.exists() else []
    elif '/' in scope:
        # Chapter like M2/C1
        return list((docs_root / scope).glob('*.md'))
    else:
        # Module like M2
        return list((docs_root / scope).rglob('*.md'))


async def main(scope: str, dry_run: bool = False):
    """Main indexing workflow."""
    # Find docs directory
    script_dir = Path(__file__).parent
    project_root = script_dir.parent.parent  # api/scripts -> project root
    docs_root = project_root / 'docs'

    if not docs_root.exists():
        print(f"Error: docs directory not found at {docs_root}")
        return

    print(f"📚 RAG Content Indexer")
    print(f"=" * 50)
    print(f"Scope: {scope}")
    print(f"Docs root: {docs_root}")
    print(f"Dry run: {dry_run}")
    print()

    # Find files
    files = find_files(scope, docs_root)
    print(f"Found {len(files)} files to index")

    if not files:
        print("No files found matching scope")
        return

    # Extract chunks from all files
    all_chunks = []
    chunk_stats = {'section': 0, 'code': 0, 'concept': 0, 'callout': 0}

    for filepath in files:
        if not filepath.exists():
            continue

        try:
            chunks = chunk_markdown(filepath, docs_root)
            all_chunks.extend(chunks)

            # Update stats
            for chunk in chunks:
                chunk_type = chunk.metadata.get('type', 'section')
                chunk_stats[chunk_type] = chunk_stats.get(chunk_type, 0) + 1

            print(f"  ✓ {filepath.name}: {len(chunks)} chunks")
        except Exception as e:
            print(f"  ✗ {filepath.name}: {e}")

    print()
    print(f"Chunk Statistics:")
    print(f"  - Sections: {chunk_stats.get('section', 0)}")
    print(f"  - Code blocks: {chunk_stats.get('code', 0)}")
    print(f"  - Concepts: {chunk_stats.get('concept', 0)}")
    print(f"  - Callouts: {chunk_stats.get('callout', 0)}")
    print(f"  - Total: {len(all_chunks)}")

    # Index to Qdrant
    await index_chunks(all_chunks, dry_run)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Index textbook content for RAG chatbot"
    )
    parser.add_argument(
        '--scope',
        default='all',
        help='Scope: "all", "M2", "M2/C1", or "M2/C1/S3"'
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Print what would be indexed without actually indexing'
    )
    args = parser.parse_args()

    asyncio.run(main(args.scope, args.dry_run))
