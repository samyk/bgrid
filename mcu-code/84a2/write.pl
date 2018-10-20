#!/usr/bin/perl

use strict;
die "usage: $0 <id>\n" unless @ARGV == 1;
my $id = shift;

if (system("platformio run -v --target upload") == 0 && $id != 0)
{
  system("eeprom_write $id");
}