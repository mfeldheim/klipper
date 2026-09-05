# Custom logo display for ST7920 displays
# This module adds logo display capability to existing ST7920 displays

import logging

class LogoDisplay:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.show_logo = config.getboolean('show_zerog_logo', False)
        
        # Register for display events
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        
    def _handle_ready(self):
        if self.show_logo:
            # Get the display object
            display = self.printer.lookup_object('display', None)
            if display and hasattr(display, 'lcd_chip'):
                self._setup_logo_display(display.lcd_chip)
                
    def _setup_logo_display(self, lcd_chip):
        """Setup logo display on the LCD chip"""
        try:
            from .zerog_logo_data import get_zerog_logo_framebuffers
            logo_framebuffers = get_zerog_logo_framebuffers()
            
            # Check if this is an ST7920-based display
            if hasattr(lcd_chip, 'graphics_framebuffers'):
                # Clear text buffer
                if hasattr(lcd_chip, 'text_framebuffer'):
                    lcd_chip.text_framebuffer[:] = b' '*64
                
                # Load the framebuffers directly
                for i, framebuffer in enumerate(logo_framebuffers):
                    if i < len(lcd_chip.graphics_framebuffers):
                        lcd_chip.graphics_framebuffers[i][:] = framebuffer
                        
                logging.info("ZeroG logo loaded successfully")
            else:
                logging.warning("Display does not support graphics mode")
                
        except ImportError:
            logging.warning("ZeroG logo data not found")
        except Exception as e:
            logging.error("Failed to load ZeroG logo: %s", e)

def load_config(config):
    return LogoDisplay(config)
