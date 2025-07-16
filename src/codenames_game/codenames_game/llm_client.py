"""
Enhanced LLM Client Module
Handles interactions with OpenAI and Anthropic APIs for the Codenames game.
Modified to support paragraph-style clues with explanations.
"""

import os
import json
import requests
from typing import List, Dict, Optional, Tuple
import logging

class EnhancedLLMClient:
    def __init__(self):
        self.openai_api_key = os.getenv('OPENAI_API_KEY', 'dummy_key')
        self.anthropic_api_key = os.getenv('ANTHROPIC_API_KEY', 'dummy_key')
        self.logger = logging.getLogger(__name__)
        
    def get_spymaster_clue_paragraph(self, target_words: List[str], avoid_words: List[str], 
                                   model: str = "gpt-4", clue_style: str = "paragraph") -> Dict[str, any]:
        """
        Generate a paragraph-style clue for the spymaster
        Returns: Dictionary with clue, count, explanation, and reasoning
        """
        
        if clue_style == "paragraph":
            prompt = f"""You are a Codenames spymaster giving a detailed clue to your operative team. You need to help them identify your team's words while avoiding opponent and dangerous words.

TARGET WORDS (your team's words): {', '.join(target_words)}
AVOID WORDS (opponent/neutral/assassin words): {', '.join(avoid_words)}

Give a detailed clue that:
1. Provides a central theme or concept that connects multiple target words
2. Explains the reasoning behind the connection
3. Gives specific hints about how many words relate to this clue
4. Warns about potential pitfalls or words to avoid
5. Uses natural, conversational language

Your clue should be 2-3 sentences that guide your operatives' thinking process.

Example format:
"Think about things related to [THEME]. I'm looking for [NUMBER] words that all connect to [EXPLANATION OF CONNECTION]. Be careful not to choose anything related to [WARNING ABOUT AVOID WORDS]."

Respond in JSON format:
{{
    "clue_paragraph": "Your detailed 2-3 sentence clue",
    "main_theme": "The central concept (1-2 words)",
    "count": number_of_related_words,
    "reasoning": "Brief explanation of why these words connect",
    "warnings": "What to avoid"
}}"""
        else:
            # Traditional single-word clue
            prompt = f"""You are a Codenames spymaster. Generate a single-word clue that relates to as many of the TARGET words as possible, while avoiding the AVOID words.

TARGET WORDS (your team's words): {', '.join(target_words)}
AVOID WORDS (opponent/neutral/assassin words): {', '.join(avoid_words)}

Rules:
1. Give exactly ONE word as your clue
2. Give a number indicating how many target words relate to your clue
3. The clue cannot be any word that appears on the board
4. The clue should relate to multiple target words if possible
5. Avoid clues that might accidentally point to avoid words

Respond in JSON format:
{{"clue": "your_single_word_clue", "count": number_of_related_words}}"""

        try:
            if "gpt" in model.lower():
                response = self._call_openai(prompt, model)
            elif "claude" in model.lower():
                response = self._call_anthropic(prompt, model)
            else:
                # Fallback to dummy response
                if clue_style == "paragraph":
                    response = '''{
                        "clue_paragraph": "Think about things you might find in nature. I'm looking for 2 words that are both living things you'd see outdoors. Be careful not to choose anything related to man-made objects.",
                        "main_theme": "NATURE",
                        "count": 2,
                        "reasoning": "Both words represent living things found in natural environments",
                        "warnings": "Avoid man-made or indoor items"
                    }'''
                else:
                    response = '{"clue": "EXAMPLE", "count": 2}'
            
            parsed = json.loads(response)
            
            if clue_style == "paragraph":
                return {
                    'clue_paragraph': parsed.get('clue_paragraph', 'Think about the connection between these words.'),
                    'main_theme': parsed.get('main_theme', 'CONNECTION'),
                    'count': parsed.get('count', 1),
                    'reasoning': parsed.get('reasoning', 'Words are related'),
                    'warnings': parsed.get('warnings', 'Be careful with your choices'),
                    'clue_type': 'paragraph'
                }
            else:
                return {
                    'clue': parsed.get('clue', 'FALLBACK'),
                    'count': parsed.get('count', 1),
                    'clue_type': 'single_word'
                }
            
        except Exception as e:
            self.logger.error(f"Error generating spymaster clue: {e}")
            if clue_style == "paragraph":
                return {
                    'clue_paragraph': "Look for words that share a common theme. I'm thinking of 1 word that fits this category. Be cautious with your selection.",
                    'main_theme': 'FALLBACK',
                    'count': 1,
                    'reasoning': 'Fallback response due to error',
                    'warnings': 'Choose carefully',
                    'clue_type': 'paragraph'
                }
            else:
                return {'clue': 'FALLBACK', 'count': 1, 'clue_type': 'single_word'}
    
    def get_operative_guesses_with_context(self, clue_data: Dict, available_words: List[str],
                                         model: str = "gpt-4") -> List[str]:
        """
        Generate guesses for the operative based on a clue (supports both paragraph and single-word clues)
        Returns: List of guessed words in priority order
        """
        
        if clue_data.get('clue_type') == 'paragraph':
            clue_paragraph = clue_data.get('clue_paragraph', '')
            main_theme = clue_data.get('main_theme', '')
            count = clue_data.get('count', 1)
            reasoning = clue_data.get('reasoning', '')
            
            prompt = f"""You are a Codenames operative. Your spymaster gave you this detailed clue:

CLUE: "{clue_paragraph}"
MAIN THEME: {main_theme}
EXPECTED COUNT: {count} words
SPYMASTER'S REASONING: {reasoning}

AVAILABLE WORDS on the board: {', '.join(available_words)}

Your task:
1. Analyze the detailed clue and understand the main theme
2. Look for words that fit the spymaster's reasoning
3. Choose up to {count + 1} words that best match the clue
4. Order them by confidence (most confident first)
5. Use the detailed explanation to guide your choices

Respond in JSON format with your guesses in order:
{{"guesses": ["word1", "word2", "word3"], "reasoning": "explanation of your thought process"}}"""
        else:
            # Handle traditional single-word clues
            clue = clue_data.get('clue', '')
            count = clue_data.get('count', 1)
            
            prompt = f"""You are a Codenames operative. Your spymaster gave you the clue "{clue}" for {count} words.

AVAILABLE WORDS on the board: {', '.join(available_words)}

Your task:
1. Think about which words relate to the clue "{clue}"
2. Choose up to {count + 1} words that best match the clue
3. Order them by confidence (most confident first)
4. You can choose fewer words if you're uncertain

Respond in JSON format with your guesses in order:
{{"guesses": ["word1", "word2", "word3"], "reasoning": "brief explanation"}}"""

        try:
            if "gpt" in model.lower():
                response = self._call_openai(prompt, model)
            elif "claude" in model.lower():
                response = self._call_anthropic(prompt, model)
            else:
                # Fallback to dummy response
                if available_words:
                    dummy_guess = available_words[0]
                    response = f'{{"guesses": ["{dummy_guess}"], "reasoning": "dummy response"}}'
                else:
                    response = '{"guesses": [], "reasoning": "no words available"}'
            
            parsed = json.loads(response)
            return parsed.get('guesses', [])
            
        except Exception as e:
            self.logger.error(f"Error generating operative guesses: {e}")
            # Return a random word as fallback
            return [available_words[0]] if available_words else []
    
    def get_spymaster_clue(self, target_words: List[str], avoid_words: List[str], 
                          model: str = "gpt-4") -> Tuple[str, int]:
        """
        Legacy method for backwards compatibility - returns single word clue
        """
        result = self.get_spymaster_clue_paragraph(target_words, avoid_words, model, "single_word")
        return result.get('clue', 'FALLBACK'), result.get('count', 1)
    
    def get_operative_guesses(self, clue: str, count: int, available_words: List[str],
                             model: str = "gpt-4") -> List[str]:
        """
        Legacy method for backwards compatibility
        """
        clue_data = {'clue': clue, 'count': count, 'clue_type': 'single_word'}
        return self.get_operative_guesses_with_context(clue_data, available_words, model)
    
    def _call_openai(self, prompt: str, model: str) -> str:
        """Call OpenAI API"""
        if self.openai_api_key == 'dummy_key':
            self.logger.warning("Using dummy OpenAI key - returning mock response")
            return '{"clue": "MOCK", "count": 1}'
        
        url = "https://api.openai.com/v1/chat/completions"
        headers = {
            "Authorization": f"Bearer {self.openai_api_key}",
            "Content-Type": "application/json"
        }
        
        data = {
            "model": model,
            "messages": [
                {"role": "system", "content": "You are a helpful assistant playing Codenames. Respond only in valid JSON format."},
                {"role": "user", "content": prompt}
            ],
            "temperature": 0.7,
            "max_tokens": 400  # Increased for paragraph responses
        }
        
        response = requests.post(url, headers=headers, json=data, timeout=30)
        response.raise_for_status()
        
        return response.json()['choices'][0]['message']['content']
    
    def _call_anthropic(self, prompt: str, model: str) -> str:
        """Call Anthropic API"""
        if self.anthropic_api_key == 'dummy_key':
            self.logger.warning("Using dummy Anthropic key - returning mock response")
            return '{"clue": "MOCK", "count": 1}'
        
        url = "https://api.anthropic.com/v1/messages"
        headers = {
            "x-api-key": self.anthropic_api_key,
            "Content-Type": "application/json",
            "anthropic-version": "2023-06-01"
        }
        
        data = {
            "model": model,
            "max_tokens": 400,
            "messages": [
                {"role": "user", "content": prompt}
            ]
        }
        
        response = requests.post(url, headers=headers, json=data, timeout=30)
        response.raise_for_status()
        
        return response.json()['content'][0]['text']

# Global instance with enhanced capabilities
enhanced_llm_client = EnhancedLLMClient()

# Backwards compatibility
llm_client = enhanced_llm_client