"""
Similarity Model Module for Codenames Game
Provides word similarity calculations using various methods including BERT embeddings.
"""

import numpy as np
import logging
from typing import List, Tuple, Dict, Optional
import os
import pickle

try:
    # Try to import sentence_transformers for BERT embeddings
    from sentence_transformers import SentenceTransformer
    SENTENCE_TRANSFORMERS_AVAILABLE = True
except ImportError:
    SENTENCE_TRANSFORMERS_AVAILABLE = False
    logging.warning("sentence-transformers not available. Using fallback similarity methods.")

try:
    # Try to import sklearn for cosine similarity
    from sklearn.metrics.pairwise import cosine_similarity
    SKLEARN_AVAILABLE = True
except ImportError:
    SKLEARN_AVAILABLE = False
    logging.warning("scikit-learn not available. Using manual cosine similarity.")

class SimilarityModel:
    """
    Word similarity model for evaluating relationships between words in Codenames.
    Supports multiple backends: BERT embeddings, word vectors, and fallback methods.
    """
    
    def __init__(self, model_name: str = "all-MiniLM-L6-v2", cache_dir: str = "./models/bert_similarity"):
        self.logger = logging.getLogger(__name__)
        self.model_name = model_name
        self.cache_dir = cache_dir
        self.model = None
        self.embedding_cache = {}
        
        # Ensure cache directory exists
        os.makedirs(cache_dir, exist_ok=True)
        
        # Initialize the model
        self._initialize_model()
        
    def _initialize_model(self):
        """Initialize the similarity model"""
        if SENTENCE_TRANSFORMERS_AVAILABLE:
            try:
                self.logger.info(f"Loading BERT model: {self.model_name}")
                self.model = SentenceTransformer(self.model_name)
                self.backend = "sentence_transformers"
                self.logger.info("✅ BERT model loaded successfully")
            except Exception as e:
                self.logger.error(f"❌ Failed to load BERT model: {e}")
                self._initialize_fallback()
        else:
            self._initialize_fallback()
    
    def _initialize_fallback(self):
        """Initialize fallback similarity methods"""
        self.backend = "fallback"
        self.logger.info("Using fallback similarity methods")
        
        # Load pre-computed word associations (simple dictionary-based)
        self.word_associations = self._load_word_associations()
    
    def _load_word_associations(self) -> Dict[str, List[str]]:
        """Load pre-computed word associations for fallback method"""
        # Simple word associations for common Codenames words
        associations = {
            "KING": ["CROWN", "ROYAL", "QUEEN", "CASTLE", "THRONE"],
            "CROWN": ["KING", "ROYAL", "QUEEN", "JEWEL", "HEAD"],
            "QUEEN": ["KING", "ROYAL", "CROWN", "CHESS", "BEE"],
            "WATER": ["RIVER", "OCEAN", "DRINK", "BLUE", "SWIM"],
            "FIRE": ["HOT", "RED", "BURN", "HEAT", "FLAME"],
            "COLD": ["ICE", "SNOW", "WINTER", "FREEZE", "BLUE"],
            "HOT": ["FIRE", "SUN", "WARM", "HEAT", "RED"],
            "ANIMAL": ["DOG", "CAT", "ZOO", "WILD", "PET"],
            "DOG": ["ANIMAL", "PET", "BARK", "PUPPY", "TAIL"],
            "CAT": ["ANIMAL", "PET", "KITTEN", "MEOW", "WHISKERS"],
            "SPACE": ["STAR", "MOON", "ROCKET", "ASTRONAUT", "GALAXY"],
            "STAR": ["SPACE", "MOON", "NIGHT", "BRIGHT", "CELEBRITY"],
            "MOON": ["SPACE", "NIGHT", "STAR", "ROUND", "CHEESE"],
            "TREE": ["FOREST", "GREEN", "WOOD", "LEAF", "BRANCH"],
            "FOREST": ["TREE", "GREEN", "WOOD", "WILD", "NATURE"],
            "HOUSE": ["HOME", "BUILDING", "ROOF", "DOOR", "FAMILY"],
            "CAR": ["DRIVE", "ROAD", "WHEEL", "ENGINE", "VEHICLE"],
            "BOOK": ["READ", "PAGE", "STORY", "LIBRARY", "PAPER"],
            "MUSIC": ["SONG", "SOUND", "INSTRUMENT", "DANCE", "BEAT"]
        }
        return associations
    
    def get_similarity(self, word1: str, word2: str) -> float:
        """
        Calculate similarity between two words.
        Returns: similarity score between 0.0 and 1.0
        """
        word1 = word1.upper().strip()
        word2 = word2.upper().strip()
        
        if word1 == word2:
            return 1.0
        
        if self.backend == "sentence_transformers":
            return self._bert_similarity(word1, word2)
        else:
            return self._fallback_similarity(word1, word2)
    
    def _bert_similarity(self, word1: str, word2: str) -> float:
        """Calculate similarity using BERT embeddings"""
        try:
            # Get embeddings
            embedding1 = self._get_embedding(word1)
            embedding2 = self._get_embedding(word2)
            
            # Calculate cosine similarity
            if SKLEARN_AVAILABLE:
                similarity = cosine_similarity([embedding1], [embedding2])[0][0]
            else:
                # Manual cosine similarity calculation
                dot_product = np.dot(embedding1, embedding2)
                norm1 = np.linalg.norm(embedding1)
                norm2 = np.linalg.norm(embedding2)
                similarity = dot_product / (norm1 * norm2)
            
            # Ensure similarity is between 0 and 1
            return max(0.0, min(1.0, float(similarity)))
            
        except Exception as e:
            self.logger.error(f"❌ Error calculating BERT similarity: {e}")
            return self._fallback_similarity(word1, word2)
    
    def _get_embedding(self, word: str) -> np.ndarray:
        """Get BERT embedding for a word (with caching)"""
        if word in self.embedding_cache:
            return self.embedding_cache[word]
        
        # Generate embedding
        embedding = self.model.encode([word])[0]
        
        # Cache the embedding
        self.embedding_cache[word] = embedding
        
        return embedding
    
    def _fallback_similarity(self, word1: str, word2: str) -> float:
        """Calculate similarity using fallback methods"""
        # Check direct associations
        if word1 in self.word_associations:
            if word2 in self.word_associations[word1]:
                return 0.8  # High similarity for direct associations
        
        if word2 in self.word_associations:
            if word1 in self.word_associations[word2]:
                return 0.8
        
        # Check mutual associations (words that appear in each other's lists)
        mutual_score = self._check_mutual_associations(word1, word2)
        if mutual_score > 0:
            return mutual_score
        
        # Simple string-based similarity
        string_sim = self._string_similarity(word1, word2)
        
        return string_sim
    
    def _check_mutual_associations(self, word1: str, word2: str) -> float:
        """Check for mutual associations between words"""
        if word1 not in self.word_associations or word2 not in self.word_associations:
            return 0.0
        
        associations1 = set(self.word_associations[word1])
        associations2 = set(self.word_associations[word2])
        
        # Check for common associations
        common = associations1.intersection(associations2)
        if common:
            return 0.6  # Medium similarity for words with common associations
        
        return 0.0
    
    def _string_similarity(self, word1: str, word2: str) -> float:
        """Simple string-based similarity using common prefixes/suffixes"""
        # Check for common prefixes
        common_prefix = 0
        min_len = min(len(word1), len(word2))
        
        for i in range(min_len):
            if word1[i] == word2[i]:
                common_prefix += 1
            else:
                break
        
        # Check for common suffixes
        common_suffix = 0
        for i in range(1, min_len + 1):
            if word1[-i] == word2[-i]:
                common_suffix += 1
            else:
                break
        
        # Calculate similarity based on common parts
        max_len = max(len(word1), len(word2))
        similarity = (common_prefix + common_suffix) / max_len
        
        # Bonus for very short words that are similar
        if min_len <= 4 and similarity > 0.5:
            similarity += 0.2
        
        return min(1.0, similarity)
    
    def get_clue_quality(self, clue: str, target_words: List[str], avoid_words: List[str]) -> Dict[str, float]:
        """
        Evaluate the quality of a clue for given target and avoid words.
        Returns: Dictionary with quality metrics
        """
        clue = clue.upper().strip()
        
        # Calculate similarities to target words
        target_similarities = [self.get_similarity(clue, word) for word in target_words]
        
        # Calculate similarities to avoid words
        avoid_similarities = [self.get_similarity(clue, word) for word in avoid_words]
        
        # Calculate metrics
        metrics = {
            "avg_target_similarity": np.mean(target_similarities) if target_similarities else 0.0,
            "max_target_similarity": max(target_similarities) if target_similarities else 0.0,
            "min_target_similarity": min(target_similarities) if target_similarities else 0.0,
            "avg_avoid_similarity": np.mean(avoid_similarities) if avoid_similarities else 0.0,
            "max_avoid_similarity": max(avoid_similarities) if avoid_similarities else 0.0,
            "num_strong_targets": sum(1 for sim in target_similarities if sim > 0.6),
            "num_risky_avoids": sum(1 for sim in avoid_similarities if sim > 0.4),
            "target_similarities": target_similarities,
            "avoid_similarities": avoid_similarities
        }
        
        # Calculate overall quality score
        target_score = metrics["avg_target_similarity"]
        avoid_penalty = metrics["max_avoid_similarity"] * 0.5
        quality_score = max(0.0, target_score - avoid_penalty)
        
        metrics["quality_score"] = quality_score
        
        return metrics
    
    def suggest_better_clues(self, target_words: List[str], avoid_words: List[str], 
                           current_clue: str = None) -> List[Tuple[str, float]]:
        """
        Suggest alternative clues based on word associations.
        Returns: List of (clue, quality_score) tuples
        """
        suggestions = []
        
        # Generate candidate clues from word associations
        candidate_clues = set()
        
        for word in target_words:
            if word in self.word_associations:
                for associated_word in self.word_associations[word]:
                    if associated_word not in target_words and associated_word not in avoid_words:
                        candidate_clues.add(associated_word)
        
        # Evaluate each candidate clue
        for clue in candidate_clues:
            if current_clue and clue.upper() == current_clue.upper():
                continue
                
            quality = self.get_clue_quality(clue, target_words, avoid_words)
            suggestions.append((clue, quality["quality_score"]))
        
        # Sort by quality score (highest first)
        suggestions.sort(key=lambda x: x[1], reverse=True)
        
        return suggestions[:10]  # Return top 10 suggestions
    
    def save_cache(self):
        """Save embedding cache to disk"""
        if self.embedding_cache:
            cache_file = os.path.join(self.cache_dir, "embedding_cache.pkl")
            try:
                with open(cache_file, 'wb') as f:
                    pickle.dump(self.embedding_cache, f)
                self.logger.info(f"💾 Saved embedding cache to {cache_file}")
            except Exception as e:
                self.logger.error(f"❌ Failed to save cache: {e}")
    
    def load_cache(self):
        """Load embedding cache from disk"""
        cache_file = os.path.join(self.cache_dir, "embedding_cache.pkl")
        if os.path.exists(cache_file):
            try:
                with open(cache_file, 'rb') as f:
                    self.embedding_cache = pickle.load(f)
                self.logger.info(f"📂 Loaded embedding cache from {cache_file}")
            except Exception as e:
                self.logger.error(f"❌ Failed to load cache: {e}")
                self.embedding_cache = {}

# Global instance for easy access
similarity_model = SimilarityModel()

def get_word_similarity(word1: str, word2: str) -> float:
    """Convenience function to get similarity between two words"""
    return similarity_model.get_similarity(word1, word2)

def evaluate_clue(clue: str, target_words: List[str], avoid_words: List[str]) -> Dict[str, float]:
    """Convenience function to evaluate a clue"""
    return similarity_model.get_clue_quality(clue, target_words, avoid_words)