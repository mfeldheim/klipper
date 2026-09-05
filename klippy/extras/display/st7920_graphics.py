# Support for ST7920 (128x64 graphics) LCD displays with full graphics mode
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2024  Custom Graphics Extension
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
from . import st7920

class ST7920Graphics(st7920.EmulatedST7920):
    def __init__(self, config):
        # Initialize the parent class first
        super().__init__(config)

        # Custom bitmap storage
        self.custom_bitmap = None
        self.show_logo = config.getboolean('show_zerog_logo', False)
        
    def get_dimensions(self):
        return (128, 64)  # Full pixel resolution
        
    def clear(self):
        if self.show_logo:
            # Load and display the ZeroG logo instead of clearing
            self.load_zerog_logo()
        else:
            # Normal clear - call parent method
            super().clear()
    def load_zerog_logo(self):
        """Load the ZeroG logo bitmap"""
        try:
            from .zerog_logo_data import get_zerog_logo_framebuffers
            logo_framebuffers = get_zerog_logo_framebuffers()

            # Clear text buffer
            self.text_framebuffer[:] = b' '*64

            # Load the framebuffers directly into the graphics framebuffers
            for i, framebuffer in enumerate(logo_framebuffers):
                if i < len(self.graphics_framebuffers):
                    self.graphics_framebuffers[i][:] = framebuffer

        except ImportError:
            logging.warning("ZeroG logo data not found, using placeholder")
            # Create a simple placeholder pattern
            for i in range(len(self.graphics_framebuffers)):
                for j in range(len(self.graphics_framebuffers[i])):
                    # Simple pattern
                    self.graphics_framebuffers[i][j] = 0x55 if (i + j) % 2 else 0xAA

    def write_text(self, x, y, data):
        """Override to prevent text when showing logo"""
        if self.show_logo:
            return  # Don't write text when showing logo
        # Normal text writing - call parent method
        super().write_text(x, y, data)
