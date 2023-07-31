#!/usr/bin/perl

use strict;

die "usage: $0 [-d (don't compile)] <id>\n" unless @ARGV;
my $dc = shift if $ARGV[0] eq "-d";
my $id = shift;

my $LFUSE = "E2"; # hex
my $SPEED = 19200;
my $CHIP = 'attiny84';
my $DEV = '/dev/tty.usbmodemFB00011';
my $CONF = "/Users/samy/.platformio/packages/tool-avrdude/avrdude.conf";
my $PROGRAMMER = "stk500v1";
# usbtiny = avr pocket programmer?
# stk500v1 = arduino as isp?
#my $AVRDUDE = "avrdude -v -p $CHIP -c $PROGRAMMER -e -D";
my $AVRDUDE = "avrdude -P$DEV -b$SPEED -v -p $CHIP -c $PROGRAMMER -e -D";
#my $AVRDUDE = "avrdude -P$DEV -b$SPEED -v -p $CHIP -C $CONF -c $PROGRAMMER -e -D";
$LFUSE = uc($LFUSE);
#my $FLASH = " -U flash:w:.pioenvs/attiny84/firmware.hex:i";
my $FW = "./.pio/build/attiny84/firmware.hex" || ".pioenvs/attiny84/firmware.hex";
my $COMPILE = "platformio run -v";

my $tmpfile = "/tmp/testeeprom.bin";
#my $flags = unpack("H2", chr(0b10000000)); # test led bit = on
my $flags = unpack("H2", chr(0b00000000)); # test led bit = off

$id = unpack("H4", pack("s>", $id));

open(OUT, ">$tmpfile") || die $!;
hout("B0 00 FF FF  FF FF FF FF    FF FF FF FF  FF FF FF FF");
hout("$id $flags");
close(OUT);

my $cmd = "$AVRDUDE -U flash:w:$FW:i -U lfuse:w:0x$LFUSE:m -U eeprom:w:$tmpfile:r";
if ($dc || run($COMPILE) == 0)
{
  run($cmd);
}
exit;


sub run
{
  my $cmd = shift;
  print "$cmd\n";
  return system($cmd);
}

sub hout
{
  my $hex = shift;
  $hex =~ s/\s//g;
  $hex = pack("H*", $hex);
  print OUT $hex;
}

__DATA__
my $CMD = "platformio run -v"; # --target upload";
open(FH, "$CMD 2>&1 |") || die $!;

my $lf;
while (<FH>)
{
  print;
  $lf = $1 if (/Fuses OK.*L:(..)/);
}
my $ret = close(FH);

# successfully ran
if ($ret == 0)
{
  if ($lf ne $LFUSE)
  {
    print "\n\nBAD FUSE! $lf != $LFUSE\n";
    system("avrdude -U lfuse:w:0xE2:m





system("avrdude -P$dev -b$speed -v -p $chip -C $conf -c $programmer -D -U eeprom:w:$tmpfile:r");


if ($lf ne "E2")

avrdude: safemode: Fuses OK (E:FF, H:DF, L:62)
while (<FH>)
{
  if (/

if (system("platformio run -v --target upload") == 0 && $id != 0)
{
  system("eeprom_write $id");
}