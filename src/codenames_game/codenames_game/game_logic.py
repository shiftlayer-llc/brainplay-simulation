"""
Codenames Game Logic Module
Core game rules, board state, and utilities for the Codenames robotics game.
"""

import random
import json
from typing import List, Dict, Tuple, Optional
from enum import Enum

class CardType(Enum):
    RED = "red"
    BLUE = "blue" 
    NEUTRAL = "neutral"
    ASSASSIN = "assassin"

class GameState(Enum):
    WAITING = "waiting"
    RED_SPYMASTER = "red_spymaster"
    RED_OPERATIVES = "red_operatives"
    BLUE_SPYMASTER = "blue_spymaster"
    BLUE_OPERATIVES = "blue_operatives"
    GAME_OVER = "game_over"

class Card:
    def __init__(self, word: str, card_type: CardType, revealed: bool = False):
        self.word = word
        self.type = card_type
        self.revealed = revealed
        
    def to_dict(self):
        return {
            'word': self.word,
            'type': self.type.value,
            'revealed': self.revealed
        }

class GameBoard:
    def __init__(self, words: List[str] = None):
        self.size = 25  # 5x5 grid
        self.cards: List[Card] = []
        self.current_state = GameState.WAITING
        self.current_team = "red"
        self.red_remaining = 9
        self.blue_remaining = 8
        self.clue_history = []
        self.guess_history = []
        
        if words:
            self.initialize_board(words)
    
    def initialize_board(self, words: List[str]):
        """Initialize the game board with random card assignments"""
        if len(words) < 25:
            raise ValueError("Need at least 25 words for the board")
        
        # Select 25 random words
        selected_words = random.sample(words, 25)
        
        # Create card types: 9 red, 8 blue, 7 neutral, 1 assassin
        card_types = (
            [CardType.RED] * 9 + 
            [CardType.BLUE] * 8 + 
            [CardType.NEUTRAL] * 7 + 
            [CardType.ASSASSIN] * 1
        )
        random.shuffle(card_types)
        
        # Create cards
        self.cards = [
            Card(word, card_type) 
            for word, card_type in zip(selected_words, card_types)
        ]
        
        self.current_state = GameState.RED_SPYMASTER
        
    def get_visible_board(self, is_spymaster: bool = False) -> List[Dict]:
        """Get board state visible to players (hide unrevealed card types for operatives)"""
        visible_cards = []
        for card in self.cards:
            card_info = {
                'word': card.word,
                'revealed': card.revealed
            }
            
            # Spymasters can see all card types, operatives only see revealed ones
            if is_spymaster or card.revealed:
                card_info['type'] = card.type.value
            else:
                card_info['type'] = 'unknown'
                
            visible_cards.append(card_info)
            
        return visible_cards
    
    def get_unrevealed_words_by_type(self, card_type: CardType) -> List[str]:
        """Get unrevealed words of a specific type (for spymaster)"""
        return [
            card.word for card in self.cards 
            if card.type == card_type and not card.revealed
        ]
    
    def make_guess(self, word: str) -> Tuple[bool, CardType, bool]:
        """
        Make a guess on a word
        Returns: (found, card_type, game_over)
        """
        for card in self.cards:
            if card.word.lower() == word.lower() and not card.revealed:
                card.revealed = True
                
                # Update remaining counts
                if card.type == CardType.RED:
                    self.red_remaining -= 1
                elif card.type == CardType.BLUE:
                    self.blue_remaining -= 1
                
                # Check win conditions
                game_over = False
                if card.type == CardType.ASSASSIN:
                    game_over = True
                    self.current_state = GameState.GAME_OVER
                elif self.red_remaining == 0:
                    game_over = True
                    self.current_state = GameState.GAME_OVER
                elif self.blue_remaining == 0:
                    game_over = True
                    self.current_state = GameState.GAME_OVER
                
                self.guess_history.append({
                    'word': word,
                    'type': card.type.value,
                    'team': self.current_team
                })
                
                return True, card.type, game_over
        
        return False, None, False
    
    def add_clue(self, clue: str, count: int, team: str):
        """Add a clue to the history"""
        self.clue_history.append({
            'clue': clue,
            'count': count,
            'team': team
        })
    
    def to_dict(self):
        """Convert board state to dictionary for JSON serialization"""
        return {
            'cards': [card.to_dict() for card in self.cards],
            'current_state': self.current_state.value,
            'current_team': self.current_team,
            'red_remaining': self.red_remaining,
            'blue_remaining': self.blue_remaining,
            'clue_history': self.clue_history,
            'guess_history': self.guess_history
        }

# Default word list for Codenames
DEFAULT_WORDS = [
    "HOLLYWOOD", "WELL", "FOOT", "NEW YORK", "SPRING", "COURT", "TUBE", "POINT", "TABLET", "SLIP",
    "DATE", "RAIL", "BREAK", "MATCH", "CRANE", "TIME", "TRAVEL", "ROBIN", "CROSS", "DOCUMENT",
    "BACK", "RIDE", "KING", "CROWN", "BRIDGE", "FALL", "POOL", "REVOLUTION", "RUSSIA", "MOUSE",
    "LONDON", "WASHINGTON", "GAME", "STAR", "BOND", "HEART", "BOW", "GHOST", "CHAMPION", "WIZARD",
    "GIANT", "HOSPITAL", "FIELD", "DRESS", "DINOSAUR", "INDIA", "KETCHUP", "ALICE", "DRAGON", "GRACE",
    "EGYPT", "MOUNT", "DEATH", "GLOVE", "DWARF", "FAN", "NURSE", "SWING", "DISEASE", "ROME",
    "PRINCESS", "PIPE", "SHOT", "PRATFALL", "TAIL", "THIEF", "BARK", "PILOT", "FACE", "SHIP",
    "ANTARCTICA", "SOUND", "MARBLE", "TOOTH", "BERLIN", "SATELLITE", "HOSPITAL", "CHURCH", "AUSTRALIA",
    "THEATER", "SMUGGLER", "BILL", "LITTER", "ROOT", "PAINT", "GOLD", "STREAM", "SAFE", "SQUARE",
    "RESEARCHER", "FAST", "BANK", "MILLIONAIRE", "OPERA", "COLD", "CAR", "NATURE", "CRUMB", "FOREST"
]

def create_default_board() -> GameBoard:
    """Create a game board with default words"""
    return GameBoard(DEFAULT_WORDS)