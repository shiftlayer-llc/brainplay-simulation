"""
LLM Client Module
Handles interactions with OpenAI and Anthropic APIs for the Codenames game.
"""

import os
import json
import requests
from typing import List, Dict, Optional, Tuple
import logging

class LLMClient:
    def __init__(self):
        self.openai_api_key = os.getenv('OPENAI_API_KEY', 'dummy_key')
        self.anthropic_api_key = os.getenv('ANTHROPIC_API_KEY', 'dummy_key')
        self.logger = logging.getLogger(__name__)
        
    def get_spymaster_clue(self, target_words: List[str], avoid_words: List[str], 
                          model: str = "gpt-4") -> Tuple[str, int]:
        """
        Generate a clue for the spymaster
        Returns: (clue, count)
        """
        
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
                response = '{"clue": "EXAMPLE", "count": 2}'
            
            parsed = json.loads(response)
            return parsed.get('clue', 'FALLBACK'), parsed.get('count', 1)
            
        except Exception as e:
            self.logger.error(f"Error generating spymaster clue: {e}")
            return "FALLBACK", 1
    
    def get_operative_guesses(self, clue: str, count: int, available_words: List[str],
                             model: str = "gpt-4") -> List[str]:
        """
        Generate guesses for the operative based on a clue
        Returns: List of guessed words in priority order
        """
        
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
                {"role": "system", "content": "You are a helpful assistant playing Codenames."},
                {"role": "user", "content": prompt}
            ],
            "temperature": 0.7,
            "max_tokens": 200
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
            "max_tokens": 200,
            "messages": [
                {"role": "user", "content": prompt}
            ]
        }
        
        response = requests.post(url, headers=headers, json=data, timeout=30)
        response.raise_for_status()
        
        return response.json()['content'][0]['text']

# Global instance
llm_client = LLMClient()