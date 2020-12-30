#ifndef __ICONS_H__
#define __ICONS_H__

#include <stdint.h>

/*
Icon Set Name: IoT Icon set 16x16 bi color
Version: 1.0.0
Origin URL:  www.engsta.com
Author: Artur Funk
License: GNU General Public License v3
*/

const uint8_t wlanIcon[] = {
    0b00000000, 0b00000000, //
    0b11100000, 0b00000111, //      ######
    0b11111000, 0b00011111, //    ##########
    0b11111100, 0b00111111, //   ############
    0b00001110, 0b01110000, //  ###        ###
    0b11100110, 0b01100111, //  ##  ######  ##
    0b11110000, 0b00001111, //     ########
    0b00011000, 0b00011000, //    ##      ##
    0b11000000, 0b00000011, //       ####
    0b11100000, 0b00000111, //      ######
    0b00100000, 0b00000100, //      #    #
    0b10000000, 0b00000001, //        ##
    0b10000000, 0b00000001, //        ##
    0b00000000, 0b00000000, //
    0b00000000, 0b00000000, //
    0b00000000, 0b00000000, //
};

const uint8_t warningIcon[] = {
    0b10000000, 0b00000000, //         #
    0b11000000, 0b00000001, //        ###
    0b11000000, 0b00000001, //        ###
    0b11100000, 0b00000011, //       #####
    0b01100000, 0b00000011, //       ## ##
    0b01110000, 0b00000111, //      ### ###
    0b00110000, 0b00000110, //      ##   ##
    0b10111000, 0b00001110, //     ### # ###
    0b10011000, 0b00001100, //     ##  #  ##
    0b10011100, 0b00011100, //    ###  #  ###
    0b10001100, 0b00011000, //    ##   #   ##
    0b00001110, 0b00111000, //   ###       ###
    0b10000110, 0b00110000, //   ##    #    ##
    0b11111111, 0b01111111, //  ###############
    0b11111111, 0b01111111, //  ###############
    0b00000000, 0b00000000, //
};

const uint8_t pressureIcon[] = {
    0b00000000, 0b00000000, //
    0b00000000, 0b00000000, //
    0b11100000, 0b00000011, //       #####
    0b11110000, 0b00000111, //      #######
    0b10011000, 0b00001100, //     ##  #  ##
    0b00111100, 0b00011000, //    ##     ####
    0b01110110, 0b00110000, //   ##     ### ##
    0b11100110, 0b00110000, //   ##    ###  ##
    0b11001110, 0b00111001, //   ###  ###  ###
    0b10000110, 0b00110000, //   ##    #    ##
    0b00000110, 0b00110000, //   ##         ##
    0b00001100, 0b00011000, //    ##       ##
    0b10011000, 0b00001100, //     ##  #  ##
    0b11110000, 0b00000111, //      #######
    0b11100000, 0b00000011, //       #####
    0b00000000, 0b00000000, //
};

const uint8_t humidityIcon[] = {
    0b00000000, 0b00000000, //
    0b10000000, 0b00000001, //        ##
    0b11000000, 0b00000011, //       ####
    0b11100000, 0b00000111, //      ######
    0b11110000, 0b00001111, //     ########
    0b11110000, 0b00001111, //     ########
    0b11111000, 0b00011111, //    ##########
    0b11011000, 0b00011111, //    ####### ##
    0b10011100, 0b00111111, //   #######  ###
    0b10011100, 0b00111111, //   #######  ###
    0b00011100, 0b00111111, //   ######   ###
    0b00111000, 0b00011110, //    ####   ###
    0b11111000, 0b00011111, //    ##########
    0b11110000, 0b00001111, //     ########
    0b11000000, 0b00000011, //       ####
    0b00000000, 0b00000000, //
};

const uint8_t temperatureIcon[] = {
    0b11000000, 0b00000001, //        ###
    0b11100000, 0b00000011, //       #####
    0b00100000, 0b00000111, //      ###  #
    0b11100000, 0b00000111, //      ######
    0b00100000, 0b00000111, //      ###  #
    0b11100000, 0b00000111, //      ######
    0b00100000, 0b00000111, //      ###  #
    0b11100000, 0b00000111, //      ######
    0b00100000, 0b00000111, //      ###  #
    0b11110000, 0b00001111, //     ########
    0b11111000, 0b00011111, //    ##########
    0b11111000, 0b00011111, //    ##########
    0b11111000, 0b00011111, //    ##########
    0b11111000, 0b00011111, //    ##########
    0b11110000, 0b00001111, //     ########
    0b11100000, 0b00000111, //      ######
};

const uint8_t connectIcon[] = {
    0b00000000, 0b00000000, //
    0b00000000, 0b00000000, //
    0b00000000, 0b00000000, //
    0b01100000, 0b00000110, //      ##  ##
    0b01100000, 0b00000110, //      ##  ##
    0b01100000, 0b00000110, //      ##  ##
    0b01100000, 0b00000110, //      ##  ##
    0b11111100, 0b00111111, //   ############
    0b11111100, 0b00111111, //   ############
    0b11111100, 0b00111111, //   ############
    0b11111100, 0b00111111, //   ############
    0b11111000, 0b00011111, //    ##########
    0b11110000, 0b00001111, //     ########
    0b11100000, 0b00000111, //      ######
    0b00000000, 0b00000000, //
    0b00000000, 0b00000000, //
};

const uint8_t iconWidth = 16;
const uint8_t iconHeight = 16;

#endif //__ICONS_H__
