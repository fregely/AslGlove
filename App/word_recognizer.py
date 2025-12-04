# word_recognizer.py
"""
Word-level ASL recognition with proper spacing and word boundaries.
"""

import time
from typing import Optional, List, Tuple
import logging

class WordRecognizer:
    """
    Manages word-level ASL recognition with proper letter spacing and word boundaries.
    """
    
    def __init__(self, logger=None):
        self.logger = logger or logging.getLogger(__name__)
        
        # Current word being spelled
        self.current_word = []
        self.completed_words = []
        
        # Timing for word boundaries
        self.last_letter_time = None
        self.word_timeout_sec = 3.0  # 3 seconds without letter = end of word
        self.letter_cooldown_sec = 1.0  # 1 second between same letter repeats
        
        # Last detected letter (for preventing duplicates)
        self.last_letter = None
        self.last_letter_timestamp = None
        
        # Recognition state
        self.is_signing = False
        self.signing_start_time = None
        
        # Statistics
        self.stats = {
            'letters_detected': 0,
            'words_completed': 0,
            'session_start': time.time()
        }
        
    def reset_current_word(self):
        """Clear the current word being spelled."""
        if self.current_word:
            self.logger.info(f"📝 Word cleared: {''.join(self.current_word)}")
        self.current_word = []
        self.last_letter = None
        self.last_letter_time = None
        
    def complete_current_word(self):
        """Mark current word as complete and move to completed list."""
        if self.current_word:
            word = ''.join(self.current_word)
            self.completed_words.append(word)
            self.stats['words_completed'] += 1
            self.logger.info(f"✅ WORD COMPLETED: {word}")
            self.logger.info(f"📝 Full sentence so far: {' '.join(self.completed_words)}")
            self.current_word = []
            self.last_letter = None
            self.last_letter_time = None
            return word
        return None
        
    def add_letter(self, letter: str, force: bool = False) -> bool:
        """
        Add a letter to the current word.
        
        Args:
            letter: Single letter to add (A-Z)
            force: If True, bypass cooldown checks
            
        Returns:
            True if letter was added, False if rejected (duplicate/cooldown)
        """
        now = time.time()
        
        # Check if same letter too soon
        if not force and letter == self.last_letter:
            if self.last_letter_timestamp:
                time_since_last = now - self.last_letter_timestamp
                if time_since_last < self.letter_cooldown_sec:
                    # Too soon for duplicate
                    return False
        
        # Add letter
        self.current_word.append(letter)
        self.last_letter = letter
        self.last_letter_time = now
        self.last_letter_timestamp = now
        self.stats['letters_detected'] += 1
        
        # Log current progress
        current = ''.join(self.current_word)
        self.logger.info(f"✉️  Letter added: {letter} → Current word: '{current}'")
        
        return True
        
    def update(self, detected_letter: Optional[str], is_stable: bool = False, 
               stability_confidence: float = 0.0) -> Tuple[Optional[str], Optional[str]]:
        """
        Update word recognition with new letter detection.
        
        Args:
            detected_letter: Letter detected this frame (or None)
            is_stable: Whether hand is stable (from ZUPT)
            stability_confidence: Confidence score for stability
            
        Returns:
            Tuple of (new_letter, completed_word)
            - new_letter: Letter just added (or None)
            - completed_word: Word just completed (or None)
        """
        now = time.time()
        new_letter = None
        completed_word = None
        
        # Check for word timeout (no letter for word_timeout_sec)
        if self.last_letter_time:
            time_since_last = now - self.last_letter_time
            if time_since_last >= self.word_timeout_sec:
                # Word timeout - complete current word
                completed_word = self.complete_current_word()
        
        # Process detected letter
        if detected_letter and is_stable and stability_confidence >= 0.90:
            # High confidence, stable detection
            if self.add_letter(detected_letter):
                new_letter = detected_letter
                
        return new_letter, completed_word
        
    def get_current_word(self) -> str:
        """Get the current word being spelled."""
        return ''.join(self.current_word)
        
    def get_completed_words(self) -> List[str]:
        """Get all completed words."""
        return self.completed_words.copy()
        
    def get_sentence(self) -> str:
        """Get the complete sentence (all words joined)."""
        return ' '.join(self.completed_words)
        
    def get_display_text(self) -> str:
        """Get formatted text for display including current and completed words."""
        parts = []
        
        # Add completed words
        if self.completed_words:
            parts.append(' '.join(self.completed_words))
            
        # Add current word being spelled
        if self.current_word:
            current = ''.join(self.current_word)
            parts.append(f"[{current}_]")  # Brackets indicate in-progress
            
        return ' '.join(parts) if parts else "[Ready to sign...]"
        
    def get_stats(self) -> dict:
        """Get recognition statistics."""
        session_time = time.time() - self.stats['session_start']
        return {
            **self.stats,
            'session_duration': session_time,
            'current_word': self.get_current_word(),
            'completed_words': len(self.completed_words),
            'letters_per_minute': (self.stats['letters_detected'] / session_time * 60) if session_time > 0 else 0
        }
        
    def backspace(self) -> bool:
        """Remove last letter from current word."""
        if self.current_word:
            removed = self.current_word.pop()
            self.logger.info(f"⬅️  Backspace: Removed '{removed}' → Current word: '{self.get_current_word()}'")
            return True
        return False
        
    def add_space(self):
        """Manually complete current word and start new one."""
        return self.complete_current_word()
        
    def clear_all(self):
        """Clear everything (current word and completed words)."""
        self.current_word = []
        self.completed_words = []
        self.last_letter = None
        self.last_letter_time = None
        self.logger.info("🗑️  All text cleared")


class SigningStateMachine:
    """
    State machine to track signing gestures and differentiate from transitions.
    """
    
    STATES = ['IDLE', 'PREPARING', 'SIGNING', 'TRANSITIONING', 'RESTING']
    
    def __init__(self):
        self.state = 'IDLE'
        self.state_entry_time = time.time()
        self.last_detection_time = None
        
        # State durations (seconds)
        self.min_signing_duration = 0.8  # Must hold letter for 800ms
        self.transition_timeout = 0.5    # 500ms of movement = transition
        self.rest_duration = 0.3         # 300ms rest between letters
        
    def update(self, is_stable: bool, letter_detected: Optional[str], 
               stability_confidence: float = 0.0) -> str:
        """
        Update state machine.
        
        Returns the current state after update.
        """
        now = time.time()
        time_in_state = now - self.state_entry_time
        
        if self.state == 'IDLE':
            if is_stable and letter_detected and stability_confidence >= 0.90:
                self._transition_to('PREPARING')
                
        elif self.state == 'PREPARING':
            if is_stable and letter_detected and stability_confidence >= 0.95:
                if time_in_state >= self.min_signing_duration:
                    self._transition_to('SIGNING')
            elif not is_stable:
                self._transition_to('TRANSITIONING')
                
        elif self.state == 'SIGNING':
            if not is_stable or stability_confidence < 0.85:
                self._transition_to('TRANSITIONING')
            # Stay in SIGNING as long as stable
                
        elif self.state == 'TRANSITIONING':
            if is_stable and stability_confidence >= 0.90:
                if letter_detected:
                    self._transition_to('PREPARING')
                else:
                    self._transition_to('RESTING')
            elif time_in_state >= self.transition_timeout:
                self._transition_to('RESTING')
                
        elif self.state == 'RESTING':
            if is_stable and letter_detected and stability_confidence >= 0.90:
                if time_in_state >= self.rest_duration:
                    self._transition_to('PREPARING')
            elif time_in_state >= 1.0:  # Max 1 second rest
                self._transition_to('IDLE')
                
        return self.state
        
    def _transition_to(self, new_state: str):
        """Internal state transition."""
        if new_state != self.state:
            self.state = new_state
            self.state_entry_time = time.time()
            
    def should_recognize(self) -> bool:
        """Returns True if currently in a state that should recognize letters."""
        return self.state == 'SIGNING'
        
    def get_time_in_state(self) -> float:
        """Get time spent in current state."""
        return time.time() - self.state_entry_time
