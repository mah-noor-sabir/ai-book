#!/usr/bin/env python3
"""
Script to export Physical AI and Humanoid Robotics book content to Qdrant vector database.
This script processes all markdown files in the docs directory and creates vector embeddings
for RAG (Retrieval Augmented Generation) purposes.
"""

import os
import re
from pathlib import Path
from typing import List, Dict, Any
import logging

from qdrant_client import QdrantClient
from qdrant_client.http import models
from sentence_transformers import SentenceTransformer
import markdown
from bs4 import BeautifulSoup


class BookExporterToQdrant:
    def __init__(self, qdrant_url: str = "localhost", qdrant_port: int = 6333):
        """
        Initialize the BookExporter with Qdrant connection details.

        Args:
            qdrant_url: URL of the Qdrant server (default: localhost)
            qdrant_port: Port of the Qdrant server (default: 6333)
        """
        self.qdrant_client = QdrantClient(host=qdrant_url, port=qdrant_port)
        self.encoder = SentenceTransformer('all-MiniLM-L6-v2')  # Lightweight model for embeddings
        self.docs_dir = Path("docs")

        # Set up logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

    def extract_content_from_md(self, file_path: Path) -> List[Dict[str, Any]]:
        """
        Extract content from a markdown file, splitting into chunks for better retrieval.

        Args:
            file_path: Path to the markdown file

        Returns:
            List of dictionaries containing content chunks with metadata
        """
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Extract frontmatter if exists
        frontmatter = {}
        content_without_frontmatter = content
        frontmatter_match = re.match(r'^---\n(.*?)\n---\n(.*)', content, re.DOTALL)
        if frontmatter_match:
            frontmatter_text = frontmatter_match.group(1)
            content_without_frontmatter = frontmatter_match.group(2)

            # Parse frontmatter (simplified - in real implementation, use yaml)
            for line in frontmatter_text.split('\n'):
                if ':' in line:
                    key, value = line.split(':', 1)
                    frontmatter[key.strip()] = value.strip().strip('"\'')

        # Convert markdown to plain text for embedding
        html = markdown.markdown(content_without_frontmatter)
        soup = BeautifulSoup(html, 'html.parser')
        plain_text = soup.get_text()

        # Split content into chunks (by paragraphs or sections)
        chunks = self._split_content(plain_text)

        # Create documents for each chunk
        documents = []
        for i, chunk in enumerate(chunks):
            if len(chunk.strip()) > 10:  # Only include substantial chunks
                document = {
                    'content': chunk,
                    'source_file': str(file_path.relative_to(self.docs_dir)),
                    'title': frontmatter.get('title', file_path.stem),
                    'section': self._get_section_from_path(file_path),
                    'chunk_id': f"{file_path.stem}_chunk_{i}",
                    'full_content': content  # Keep original for context
                }
                documents.append(document)

        return documents

    def _split_content(self, text: str, max_chunk_size: int = 1000) -> List[str]:
        """
        Split text content into chunks of appropriate size for embedding.

        Args:
            text: Text to split
            max_chunk_size: Maximum size of each chunk

        Returns:
            List of text chunks
        """
        # Split by paragraphs first
        paragraphs = [p.strip() for p in text.split('\n\n') if p.strip()]

        chunks = []
        current_chunk = ""

        for paragraph in paragraphs:
            if len(current_chunk) + len(paragraph) < max_chunk_size:
                current_chunk += "\n\n" + paragraph
            else:
                if current_chunk:
                    chunks.append(current_chunk.strip())
                current_chunk = paragraph

        if current_chunk:
            chunks.append(current_chunk.strip())

        # If any chunk is still too large, split by sentences
        final_chunks = []
        for chunk in chunks:
            if len(chunk) > max_chunk_size:
                sentences = re.split(r'[.!?]+', chunk)
                temp_chunk = ""
                for sentence in sentences:
                    sentence = sentence.strip()
                    if len(temp_chunk) + len(sentence) < max_chunk_size:
                        temp_chunk += " " + sentence
                    else:
                        if temp_chunk.strip():
                            final_chunks.append(temp_chunk.strip())
                        temp_chunk = sentence
                if temp_chunk.strip():
                    final_chunks.append(temp_chunk.strip())
            else:
                final_chunks.append(chunk)

        return [chunk for chunk in final_chunks if len(chunk) > 10]

    def _get_section_from_path(self, file_path: Path) -> str:
        """
        Extract section/module name from file path.

        Args:
            file_path: Path to the markdown file

        Returns:
            Section name
        """
        parts = file_path.parts
        for part in parts:
            if 'Module' in part:
                return part
        return 'Introduction'

    def get_all_markdown_files(self) -> List[Path]:
        """
        Get all markdown files from the docs directory.

        Returns:
            List of markdown file paths
        """
        md_files = list(self.docs_dir.rglob("*.md"))
        self.logger.info(f"Found {len(md_files)} markdown files to process")
        return md_files

    def create_collection(self, collection_name: str = "physical_ai_book"):
        """
        Create a Qdrant collection for storing book content.

        Args:
            collection_name: Name of the collection to create
        """
        # Delete collection if it already exists
        try:
            self.qdrant_client.delete_collection(collection_name)
            self.logger.info(f"Deleted existing collection: {collection_name}")
        except:
            pass  # Collection doesn't exist, which is fine

        # Create new collection with appropriate vector size (384 for all-MiniLM-L6-v2)
        self.qdrant_client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(
                size=384,  # Size of the sentence transformer embeddings
                distance=models.Distance.COSINE
            )
        )
        self.logger.info(f"Created collection: {collection_name}")

    def export_to_qdrant(self, collection_name: str = "physical_ai_book"):
        """
        Export all book content to Qdrant vector database.

        Args:
            collection_name: Name of the Qdrant collection to store the data
        """
        self.logger.info("Starting export to Qdrant...")

        # Create the collection
        self.create_collection(collection_name)

        # Get all markdown files
        md_files = self.get_all_markdown_files()

        # Process each file and add to Qdrant
        all_documents = []
        for file_path in md_files:
            self.logger.info(f"Processing file: {file_path}")
            try:
                documents = self.extract_content_from_md(file_path)
                all_documents.extend(documents)
                self.logger.info(f"  Extracted {len(documents)} chunks from {file_path.name}")
            except Exception as e:
                self.logger.error(f"Error processing {file_path}: {e}")

        # Create embeddings and upload to Qdrant
        self.logger.info(f"Creating embeddings for {len(all_documents)} document chunks...")

        points = []
        for i, doc in enumerate(all_documents):
            # Create embedding for the content
            embedding = self.encoder.encode(doc['content']).tolist()

            # Create a Qdrant point
            point = models.PointStruct(
                id=i,
                vector=embedding,
                payload={
                    'content': doc['content'],
                    'source_file': doc['source_file'],
                    'title': doc['title'],
                    'section': doc['section'],
                    'chunk_id': doc['chunk_id']
                }
            )
            points.append(point)

            # Upload in batches to be efficient
            if len(points) >= 100:  # Batch size of 100
                self.qdrant_client.upsert(collection_name=collection_name, points=points)
                self.logger.info(f"Uploaded batch of {len(points)} points to Qdrant")
                points = []  # Reset for next batch

        # Upload remaining points
        if points:
            self.qdrant_client.upsert(collection_name=collection_name, points=points)
            self.logger.info(f"Uploaded final batch of {len(points)} points to Qdrant")

        self.logger.info(f"Successfully exported {len(all_documents)} document chunks to Qdrant collection '{collection_name}'")

    def search_in_qdrant(self, query: str, collection_name: str = "physical_ai_book", limit: int = 5):
        """
        Search for content in the Qdrant database.

        Args:
            query: Search query
            collection_name: Name of the collection to search
            limit: Number of results to return

        Returns:
            List of search results
        """
        query_embedding = self.encoder.encode(query).tolist()

        search_results = self.qdrant_client.search(
            collection_name=collection_name,
            query_vector=query_embedding,
            limit=limit
        )

        return search_results


def main():
    """
    Main function to run the export process.
    """
    exporter = BookExporterToQdrant()

    print("Exporting Physical AI and Humanoid Robotics book to Qdrant...")
    print("This will:")
    print("1. Read all markdown files from the docs directory")
    print("2. Split content into chunks")
    print("3. Create embeddings using sentence transformers")
    print("4. Store in Qdrant vector database")

    exporter.export_to_qdrant()

    print("\nExport completed successfully!")

    # Example search to verify the data was imported
    print("\nTesting search functionality...")
    results = exporter.search_in_qdrant("embodied intelligence", limit=3)

    print(f"\nTop 3 results for 'embodied intelligence':")
    for i, result in enumerate(results, 1):
        content_preview = result.payload['content'][:200] + "..." if len(result.payload['content']) > 200 else result.payload['content']
        print(f"{i}. {result.payload['title']} ({result.payload['section']})")
        print(f"   Score: {result.score:.3f}")
        print(f"   Content: {content_preview}")
        print()


if __name__ == "__main__":
    main()