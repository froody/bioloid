#! /usr/bin/perl
use strict;
use IO::Socket;

# yum install perl-TermReadKey
use Term::ReadKey qw();


my %def=();

unless(open(FCONST,'../../main/include/constants.h')) {
 die('Could not load constants.h file');
}
else {
 while(my $line=<FCONST>) {
  $line =~ s{[\n\r\t]}{}g;
  $line =~ s{\s*//.*$}{};  # Strip off trailing comments
  next unless($line =~ m{^\#define\s+(\S+)\s+(.+)$});
  my ($key, $value)=($1, $2);
  $value=hex($value) if($value =~ m{0x});
  $value=$1 if($value =~ m{\((\d+)\)});
  $def{$key}=$value;
 }
# print map { "'$_' -> '$def{$_}'\n" } sort keys %def;
 close(FCONST);
}

my $sock = new IO::Socket::INET (
 PeerAddr => 'localhost',
 PeerPort => '7777',
 Proto => 'tcp',
);
die "Error: $!\n" unless $sock;
$sock->autoflush(1);
binmode $sock;

#SendCommand(1, 1, [ pack('n', 456), ]);

#Term::ReadKey::ReadMode('normal'); # Wait for line
#Term::ReadKey::ReadMode('noecho'); # Wait for line, no-echo
#Term::ReadKey::ReadMode('raw');    # Raw capture of keys (including Cntrl-C)
Term::ReadKey::ReadMode('cbreak');  # Raw capture of keys (but break on Cntrl-C)

$|=1; # Immediate printing flush (don't wait for newline)

my $servo=1;
my $mode='';
# $mode='comms';
$mode='step';

my $imu_id=100;
my $imu_read=1;

my $tilt=0;

my $next_position=450;
my $alternate=1;

while(1) { 
 print '.'; 
 my $key=Term::ReadKey::ReadKey(0.5); # Read in 0.5sec intervals
 if($mode eq 'step' && $imu_read) {
  SendCommand($imu_id,   $def{INST_READ}, [ $def{P_IMUHUV_SIDEWAYS_ACCEL_L}, 6 ]);
  my $ret=GetReturn();
  my @accel=unpack('CCCCCC', $$ret{param}); # 3 LH pairs
  printf "Accel : (%5.2f, %5.2f, %5.2f)\n",
      LHtoAccel($accel[0],$accel[1]), 
      LHtoAccel($accel[2],$accel[3]), 
      LHtoAccel($accel[4],$accel[5]);
 }
 
 next unless(defined $key);

 print "\nGot '$key'\n";
 unless($mode) { # Mode is not set - choose it based on key
  $mode='comms' if($key eq 'c');
  $mode='step'  if($key eq 's');

  last if($key eq 'q');
  next;
 }
 if($mode eq 'comms') {
  if($key eq 'q') {
   print "Exiting Communications Test mode\n";
   $mode='';
   next;
  }
  if($key eq '1') {
   print "Reset Bioloid device 1\n";
   SendCommand(1, $def{INST_RESET}, []);
  }

  if($key eq '2') {
   print "Ping Bioloid device 2\n";
   SendCommand(2, $def{INST_PING}, []);
   GetReturn();
  }
  
  if($key eq '3') {
   my $p=400;
   print "Move the Current Servo ($servo) to $p\n";
   SendCommand($servo, $def{INST_WRITE}, [ $def{P_GOAL_POSITION_L}, Int16ToLH($p), ]);
   GetReturn();
  }
  if($key eq '4') {
   my $p=$next_position;
   print "Flip Current Servo ($servo) to $p\n";
   
   SendCommand($servo, $def{INST_WRITE}, [ $def{P_GOAL_POSITION_L}, Int16ToLH($p), ]); 
   GetReturn();

   $next_position=512+(512-$next_position);
  }

  if($key eq '5') {
   $servo--;
   print "Decrement Servo (now $servo)\n";
  }
  if($key eq '6') {
   $servo++;
   print "Increment Servo (now $servo)\n";
  }
  
  if($key eq '7') {
   my $p=$next_position;
   print "Prepare to Move two servos simultaneously to $p\n";
   
   # Set up the servos to respond be ready to execute
   SendCommand($servo,   $def{INST_REG_WRITE}, [ $def{P_GOAL_POSITION_L}, Int16ToLH($p), ]);
   GetReturn();
   SendCommand($servo+1, $def{INST_REG_WRITE}, [ $def{P_GOAL_POSITION_L}, Int16ToLH($p), ]);
   GetReturn();
   
   $next_position=512+(512-$next_position);
  }
  
  if($key eq '8') {
   print "Broadcast ACTION to trigger momements\n";
   SendCommand($def{BROADCASTING_ID},   $def{INST_ACTION}, [ ]);
  }

  if($key eq '9') {
   my $p=$next_position;
   print "Move two servos simultaneously with one message to : $p\n";
   
   SendCommand($def{BROADCASTING_ID},   $def{INST_SYNC_WRITE},  [ 
    $def{P_GOAL_POSITION_L}, # Start Address to write to
    2,  #  Length of data (not including servo #)
    $servo, Int16ToLH($p), 
    $servo+1, Int16ToLH($p), 
   ]);
   
   $next_position=512+(512-$next_position);
  }

  if($key eq '0') {
   print "Zero all servos\n";
   MoveTo([ (1 .. 18) ],[ (0)x18 ]);
  }
 }	

  if($mode eq 'step') {
  if($key eq 'q') {
   print "Exiting Stepping mode\n";
   $mode='';
   next;
  }
  if($key eq '1') {  # Tilt forward by moving ankles
   my $angle=5;
   MoveTo([ 15,16 ],[ $angle, -$angle]);
  }
  if($key eq '2') {  # 'Boogie' to RHS/LHS by tilting ankles and hips
   my $angle=$alternate*5;
   MoveTo([ 9,10, 17,18 ],[ $angle,$angle, $angle,$angle ]);
   $alternate=-$alternate;
  }
  if($key eq '3') {  # Swing the legs (don't alternate)
   my $size=20;
   my $angle1=-$alternate*$size+$size;
   my $angle2=$alternate*$size+$size;
#			MoveTo([ 11,12, 13,14, ],[ $angle,$angle, $angle,$angle  ]);
   MoveTo([ 11, 13, ],[ -$angle1,-$angle1  ]);
   MoveTo([ 12, 14, ],[  $angle2, $angle2  ]);
  }
  if($key eq '4') {  # Tilt sideways onto one leg
   $tilt+=1;
   MoveTo([ 9,17,18,1,3 ],[ -$tilt, $tilt, -$tilt, 90,-80 ]);
  }
  
  
  
 }

 
}

Term::ReadKey::ReadMode('restore'); 
print "\nEnd\n";

close($sock);
exit 0;

sub SendCommand {
 my ($id, $instr, $params)=@_;
 my $len=@$params + 2; # Add instruction and checksum to parameter count
 
#LINE 1  DIRECTION_PORT = TX_DIRECTION;
#LINE 2  TxDByte(0xff);
#LINE 3  TxDByte(0xff);
#LINE 4  TxDByte(bID);
#LINE 5  TxDByte(bLength);
#LINE 6  TxDByte(bInstruction);
#LINE 7  TxDByte(Parameter0); TxDByte(Parameter1); ...
#LINE 9  TxDByte(Checksum); //last TxD

 my $header=pack('CCCCC', 0xFF,0xFF, $id,$len,$instr);
 my $checksum=$id+$len+$instr;
 foreach my $c (@$params) {
  $checksum += $c;
 }
 
 my $send=$header.pack('C*', @$params, (~$checksum) & 0xFF);

 print "Sending  : ", (map { sprintf('%.2X ', $_); } unpack('C*', $send) ), "\n";
 print $sock $send;
 # flush socket now (?)
}

sub GetReturn {
 my ($header, $param);
 return(undef) unless(read($sock, $header, 5) == 5);
 print "Received : ", (map { sprintf('%.2X ', $_); } unpack('C*', $header) );

 my @header=unpack('C*', $header);
 return(undef) unless($header[0]==0xFF && $header[1]==0xFF);

 my $len=$header[3];
 my $status={ 
  id=>$header[2], 
  err=>$header[4],
  param=>'',
 };
 if($len>1) {
  return(undef) unless(read($sock, $param, $len-1) == $len-1);
  print "{", (map { sprintf('%.2X ', $_); } unpack('C*', $param) ), "}";
  my $sum=0;
  map { $sum+=$_; } unpack('C*', substr($header,2,5).$param);
  unless(($sum & 0xFF) == 0xFF) {
   print "CRC Failure!\n";
   return undef;
  }
  $$status{param}=substr($param, 0, length($param)-1);
 }
 print "\n";
 
 return $status;
}

sub Int16ToLH {
 return ($_[0] % 256, $_[0] >> 8);
}

sub DegreesToLH {
 return Int16ToLH($_[0]/300 * 1023 + 512);
}

sub LHtoInt16 {
 return $_[0] + ($_[1] << 8);
}

sub LHtoAccel {
 return (LHtoInt16(@_)-512)/512*3; 
}

sub MoveTo { 
 my ($servos, $angles)=@_; 
 my @params=(
  $def{P_GOAL_POSITION_L}, # Start Address to write to
  2,  #  Length of data (not including servo #)
  );
 
 for(my $i=0; $i<@$servos; $i++) {
  push @params, $$servos[$i];
  push @params, DegreesToLH($$angles[$i]);
 }
 SendCommand($def{BROADCASTING_ID},   $def{INST_SYNC_WRITE},  \@params);
}


__END__
LINE 1  DIRECTION_PORT = TX_DIRECTION;
LINE 2  TxDByte(0xff);
LINE 3  TxDByte(0xff);
LINE 4  TxDByte(bID);
LINE 5  TxDByte(bLength);
LINE 6  TxDByte(bInstruction);
LINE 7  TxDByte(Parameter0); TxDByte(Parameter1); ...
LINE 8  DisableInterrupt(); // interrupt should be disable
LINE 9  TxDByte(Checksum); //last TxD
LINE 10 while(!TXD_SHIFT_REGISTER_EMPTY_BIT); //Wait till last data bit has been sent
LINE 11 DIRECTION_PORT = RX_DIRECTION; //Direction change to RXD
LINE 12 EnableInterrupt(); // enable interrupt again

Please note the important lines between LINE 8 and LINE 12. Line 8 is necessary since
an interrupt here may cause a delay longer than the return delay time and corruption to
the front of the status packet may occur.
