#! /bin/perl -w
use strict;

# perl perl-to-xml.pl > ../physics_humanoid.xml

use Data::Dumper;
$Data::Dumper::Indent=1; # Simple, fixed

my %robot=();
my $inverse_hip=0;  # Set to 1 for inverse hip magic / 0 for Humanoid defined in Bioloid manual

if(1) {
 local $/=undef;
 if(open(TXT, "<PortugeseHumanoid.txt")) {
  my $def=<TXT>;
  close(TXT);
  eval($def);
  if($@) {
   die($@);
  }
  $def =~ s{\t}{\n}g;
  $def =~ s{\n+}{\n}g;
#		warn $def;
 }
}
if(0) {
 $robot{Thigh}={
  CoG=>{
   position=>{
    'world'=>{ xyz=>[1,2,3], },
   },
  },
 };
 $robot{Shin}={
  CoG=>{
   position=>{
    'ThighCoG'=>{ xyz=>[0,2,-5], },
   },
  },
 };
 $robot{Foot}={
  name=>'Foot',
  CoG=>{
   position=>{
    'ShinCoG'=>{ xyz=>[0,3,-10], },
   },
  },
 };
} # end if()

#warn Dumper(\%robot);
#exit 0;

if($inverse_hip) {  # Flip over some of the reference read from the file...
 my $p={};
  
 $p=$robot{GroinRight}{MidBracket}{position};
 $$p{GroinRightMidBracketInv}=$$p{GroinRightMidBracketReg};
 delete $$p{GroinRightMidBracketReg};

 $p=$robot{GroinLeft}{MidBracket}{position};
 $$p{GroinLeftMidBracketInv}=$$p{GroinLeftMidBracketReg};
 delete $$p{GroinLeftMidBracketReg};
 
 $p=$robot{Torso}{RightLegTwist}{position};
 $$p{TorsoRightLegTwistInv}{xyz}=$$p{TorsoRightLegTwistReg}{xyz};
 delete $$p{TorsoRightLegTwistReg}{xyz};

 $p=$robot{Torso}{LeftLegTwist}{position};
 $$p{TorsoLeftLegTwistInv}{xyz}=$$p{TorsoLeftLegTwistReg}{xyz};
 delete $$p{TorsoLeftLegTwistReg}{xyz};
}


my @xml=();

# Now, Name all the hashes nicely
# Don't name the 'lower case intermediaries' structures, though
iterate_through_structure(\%robot,  
 sub {
  my ($parent, $key)=@_; 
  return unless (ref($$parent{$key}) eq 'HASH'); # Nothing to do for this one
  my $name=$key;
  my $element='name';
  if(lc($name) eq $name) { # All in lower case?  Then set _name instead - and omit the key here
   $element='_name';
   $name='';
  }
  $$parent{$key}{$element} ||= ($$parent{name} || $$parent{_name} || '').$name;
 }
);
#warn Dumper(\%robot);

my $world_xyz={};      # This is currently in the frame of the file we've read in
my $world_inertia={};  # This is currently in the frame of the file we've read in

# Find all the positions without world coordinates, and resolve them
my ($unresolved, $discovered_something)=('World',0); # Force first iteration
while($unresolved) {
 $unresolved=''; # Try to be optimistic
 $discovered_something=0; # But stop if we don't resolve anything in an iteration
 iterate_through_structure(\%robot, 
  sub {
   my ($parent, $key)=@_;
   my $ref=$$parent{$key};
   if($key eq 'position') {
#    print "Examining $$parent{name} : ", join(', ', keys %$ref), "\n";
    if(exists $$ref{world}) {
     unless($$world_xyz{$$parent{name}}) { # unless it's there already...
      $$world_xyz{$$parent{name}}=$$ref{world}; # Store the known data
     }
    }
    else {
     foreach my $k (keys %$ref) { # See whether we have a known frame of reference
      next if($k eq 'name' || $k eq '_name');
#						warn "Is $k known? : ".((exists $$world_xyz{$k})?'Yes':'No')."\n";
      if(exists $$world_xyz{$k}) {
       my $world_xyz=$$world_xyz{$k}{xyz};
       my $rel_xyz  =$$ref{$k}{xyz}; 
       my @xyz=();
       for(my $i=0; $i<3; $i++) {
        $xyz[$i]=($$world_xyz[$i] || 0)+($$rel_xyz[$i] || 0);
       }
       $$ref{world}={ 
        xyz=>\@xyz,
       };
       $discovered_something=1;
      }
      else {
       $unresolved=$k; # Need to re-iterate
      }
     }
     $unresolved ||= 'Something' ; # Need to re-iterate
    }
   }
   if($key eq 'inertia') { #  Just store these - these are correct in the file's frame already
    if(ref($ref)) { # Ignore our scaling one
     $$world_inertia{$$parent{name}}=$ref;
    }
   }
  }
 );
 unless($discovered_something || !$unresolved) {
  die "Couldn't resolve model - there's a dangling reference to '$unresolved'\n";
 }
}


# $world_xyz      ::    # Now transform this to the standard OpenGL style frame
# $world_inertia  ::    # Now transform this to the standard OpenGL style frame

# If the coordinate system needs changing to be the (assumed) :
$robot{transform} ||= [[1,0,0], [0,1,0], [0,0,1]]; # Default is no transform
# $robot{transform} = [[1,0,0], [0,1,0], [0,0,1]]; # Default is no transform

# $robot{scale} ||= { length_to_m=>1, mass_to_kg=>1, inertia_to_si=>1, };
my %scale=( length_to_m=>1, mass_to_kg=>1, inertia_to_si=>1, %{$robot{scale} || {}},  );

# OpenGL frame (?) : X back, Y right, Z up (x~depth, y~width, z~height)
# Do it first - before anything else occurs...
#if(1) {
# iterate_through_structure(\%robot, 
#  sub {
#   my ($parent, $key)=@_;
#			my $ref=$$parent{$key};
#			if($key eq 'position') { # These positions need adjusting for sure
##    warn "Examining $$parent{name} : ", join(', ', keys %$ref), "\n";
#				foreach my $k (keys %$ref) { # See whether we have a known frame of reference
#					next if($k eq 'name' || $k eq '_name');
#					$$ref{$k}{xyz}=multiply_vector3($$ref{$k}{xyz}, $robot{transform});
#				}
#			}
#			if($key eq 'inertia') { # The inertia matrix also needs transforming
#				warn "parent{name}='$$parent{name}'\n";
#    warn "Examining $$parent{name} : [", join(', ', map { "[$$_[0],$$_[1],$$_[2]]" } @$ref), "];\n";
#				$ref=multiply_matrix3($ref, $robot{transform});
#			}
#		}
#	);
#}

foreach my $loc (keys %$world_xyz) {
 $$world_xyz{$loc}{xyz}=multiply_matrix_by_vector3($robot{transform}, $$world_xyz{$loc}{xyz});
}
foreach my $b (keys %$world_inertia) {
 $$world_inertia{$b}=multiply_matrix_by_matrix3(
                      transpose_matrix3($robot{transform}), 
                      multiply_matrix_by_matrix3($$world_inertia{$b}, $robot{transform})
                     );
}

# Show the coordinates found
if(0) {
 foreach my $p (sort { $$world_xyz{$a}{xyz}[2] <=> $$world_xyz{$b}{xyz}[2] } keys %$world_xyz) {
  warn "Position{$p}=[".join(',', @{$$world_xyz{$p}{xyz}})."]\n";
 }
}

#warn Dumper(\%robot);
#exit 0;

# Take out temporary bodies
my @all_bodies=();
foreach my $body (sort keys %robot) {
 next if(lc($body) eq $body); # Ignore all-lower-case bodies
 push @all_bodies, $body;
}

# Find the lowest point in the model (min(z)), so we can shift the robot entirely above the ground
my $min_z=0;
foreach my $loc (keys %$world_xyz) {
#	test_exists(\%robot, "{$body}{CoG}{position}{world}{xyz}");
# my $pos =$robot{$body}{CoG}{position}{world}{xyz};
#	test_exists(\%robot, "{$body}{CoG}{name}");
# my $pos = $$world_xyz{$robot{$body}{CoG}{name}}{xyz};
 my $pos = $$world_xyz{$loc}{xyz};
 if($min_z>$$pos[2]) {
  $min_z=$$pos[2];
 }
}

# $min_z-=   100; # Take off 100mm from every z-coordinate... : 10cm - allows a short fall to start
# $min_z-=   5; # Take off 5mm from every z-coordinate...

# for a fall time t : s=0.t+0.5*9.876*t*t =~ 5* t*t
# v=u+at : v =~ 10t 
# $min_z-=20_000; # Take off 20m from every z-coordinate...  => fall time of 2sec

# and then do so...
foreach my $loc (keys %$world_xyz) {
 my $pos = $$world_xyz{$loc}{xyz};
 $$pos[2]-=$min_z;
}




# Show bodies sorted by Height (Z coord) of CoG (has no temporary entries in it either)
my @cog_order=sort {
 $$world_xyz{$robot{$a}{CoG}{name}}{xyz}[2] <=> $$world_xyz{$robot{$b}{CoG}{name}}{xyz}[2] 
} @all_bodies;

if(0) {  # Show the inertias found
 foreach my $p (@cog_order) {
  warn substr("Inertia{$p}=".(' 'x100), 0, 25).'['.join(', ', map { sprintf('[%7.2f,%7.2f,%7.2f]',$$_[0],$$_[1],$$_[2]); } @{$$world_inertia{$p}} )."];\n";
 }
}
foreach my $p (@cog_order) {
 my $m=$$world_inertia{$p};
 unless($$m[0][1]==$$m[1][0] && $$m[0][2]==$$m[2][0] && $$m[2][1]==$$m[1][2]) {
  warn "Non-symmetric Inertia tensor for '$p'\n";
 }
}


# Denavit Hartenberg parameters: 
#rotz = Rotation around z-axis, 
#transz = Translation along z-axis, 
#rotx = Rotation around x'-axis, 
#transx = Translation along x'-axis, 
#id = Id of Ax12-servo assigned to joint, 
#sgn = direction of rotation (+/- 1)

# 
# The transformation matrices are given by
# A[n] = Rot(theta[n] about z[n]) * Trans(d[n] along z[n])
#       * Trans(a[n] along x[n]) * Rot(alpha[n] about x[n])

# A[n] = Rot(rotz[n] about z[n]) * Trans(transz[n] along z[n])
#       * Trans(transx[n] along x[n]) * Rot(rotx[n] about x[n])
#  mdda thinks : === 
# A[n] = Trans(transz[n] along z[n]) * Rot(rotz[n] about z[n])
#       * Trans(transx[n] along x[n]) * Rot(rotx[n] about x[n])


# Ok, so now let's figure out the joints...






## Ok, so now let's output some XML

my $humanoid_name='HUMANOID'.($inverse_hip?'_INVERSE_HIP':'');
push @xml, qq(<?xml version="1.1" ?>
<Robot>
 <Name>$humanoid_name</Name>
 <UseLoadFunction>false</UseLoadFunction>);

my $units=sprintf(qq(length_to_m="%10.8f" mass_to_kg="%10.8f" inertia_to_si="%10.8f"), $scale{length_to_m}, $scale{mass_to_kg}, $scale{inertia_to_si});
push @xml, qq(
 <physics $units>);

foreach my $body (@cog_order) {
 my $base=$robot{$body}{base} || $body;

#	next unless($body =~ 'Arm|Torso');  
#	next unless($body =~ 'Torso|Hip');  
#	next unless($body =~ 'Hip|LowerLeg');
#	next unless($body =~ 'LowerLeg|Ankle|Foot');
#	next unless($body =~ 'Ankle|LowerLeg');
#	next unless($body =~ 'LowerLeg');
#	next unless($body =~ 'LowerLegLeft');
 
 push @xml, qq(
  <body body="$body" base="$base">);

#	warn "Looking up name '$robot{$body}{CoG}{name}'\n";
 my $pos=$$world_xyz{$robot{$body}{CoG}{name}}{xyz};
 my ($x,$y,$z)=@$pos;

 my $mass=$robot{$body}{mass} || 0;
 push @xml, qq(
   <cog x="$x" y="$y" z="$z" mass="$mass" />);
   
 my $i=$$world_inertia{$body} || [[1,0,0],[0,1,0],[0,0,1]];
 push @xml, qq(
   <inertia xx="$$i[0][0]" xy="$$i[0][1]" xz="$$i[0][2]" yx="$$i[1][0]" yy="$$i[1][1]" yz="$$i[1][2]" zx="$$i[2][0]" zy="$$i[2][1]" zz="$$i[2][2]" />);

 push @xml, qq(
  </body>);
}

# Ok, so now let's print out the joints...
my %electronics=();

my $joints=$robot{joints} || {};
foreach my $j (sort keys %$joints) {  # { $$joints{$a}{name} cmp $$joints{$b}{name} } 
 next if(lc($j) eq $j); # Ignore all-lower-case bodies
#	warn "Joint : $j\n";
  my $joins =$$joints{$j}{joins} || '';
 next unless($joins); # Skip the Torso itself (part of chains, but not a joint location per-se)
 
 my $servo=$$joints{$j}{servo} || 0;
 my $axis_through_wheel=$$joints{$j}{sense} || +1;  # This is on the AX12 diagrams
 
 my $origin=$$joints{$j}{origin} || '';
 my $axle=$$joints{$j}{axle} || '';

  my $center=$$world_xyz{$j.$origin}{xyz} || [0,0,0];
 my @axis=(0,0,1);
  if(my $a=$$world_xyz{$j.$axle}{xyz}) {
  @axis=map { $$center[$_]-$$a[$_]; } (0 .. 2);
 }
 if($axis_through_wheel<0) {  # Flip the direction if Centre->Axle points out of the servo at the opposite side to the wheel
  @axis=map { -$_; } @axis;
 }

 my $name  = $$joints{$j}{name} || "$j - $joins";  # $$joints{$j}{name} || 

 push @xml,  qq(
  <joint name="$name" body1="$j" body2="$joins">
   <center x="$$center[0]" y="$$center[1]" z="$$center[2]" />
   <axis x="$axis[0]" y="$axis[1]" z="$axis[2]" />
  </joint>);

 my $initial_degrees=0;
 
 $initial_degrees=+12 if($name =~ m{LegSwing$});
 $initial_degrees=+20 if($name =~ m{Knee$});
 $initial_degrees=-10 if($name =~ m{Ankle$});
 
 $initial_degrees = -$initial_degrees if($name =~ m{Left});
 
 my $zero=($initial_degrees)?"initial_degrees=$initial_degrees ":'';
 
 $electronics{$servo}=qq(
  <servo id="$servo" name=").$name.qq(Servo" type="ax12">
   <attached joint="$name" $zero/>
  </servo>);
}

push @xml, qq(
 </physics>);
 
# Add IMUs... 
if(1) { 
 my $id=100; # Standard HVU IMU has $id=120
 $electronics{$id}=qq(
  <sensor id="$id" name="TorsoIMU" type="imu" version="ideal">
   <attached body="Torso" />
  </sensor>);
}

# Add Foot Pressure Sensor... 
if(0) { 
 $electronics{121}=qq(
  <sensor id="121" name="LeftFootPressure" type="pressure" version="huv">
   <attached body="LeftFoot" />
  </sensor>);
 $electronics{122}=qq(
  <sensor id="122" name="RightFootPressure" type="pressure" version="huv">
   <attached body="RightFoot" />
  </sensor>);
}

# Add AXS1 - sensor
if(0) { 
 $electronics{64}=qq(
  <sensor id="64" name="TorsoAXS1" type="various" version="axs1" >
   <attached body="Torso" />
  </sensor>);
}


push @xml, qq(
 <electronics>);
foreach my $id (sort { $a <=> $b } keys %electronics) {
  push @xml, $electronics{$id};	
}
push @xml, qq(
 </electronics>);
 
push @xml, qq(
 <geometry>);

foreach my $body (@cog_order) {
 next unless(exists $robot{$body}{geom});

 my $geoms=$robot{$body}{geom};
 foreach my $geom (keys %$geoms) {
  next if(lc($geom) eq $geom); # Skip the all-lowercase entities

#		warn "Doing geom = '$body-$geom'\n";
  my $base=$robot{$body}{base} || $body;

  test_exists(\%robot, "{$body}{geom}{$geom}{Corner1}{name}");
  test_exists(\%robot, "{$body}{geom}{$geom}{Corner2}{name}");
  my $c1=$$world_xyz{$robot{$body}{geom}{$geom}{Corner1}{name}}{xyz};
  my $c2=$$world_xyz{$robot{$body}{geom}{$geom}{Corner2}{name}}{xyz};

  warn "Undefined values on : '$body' - '$geom'\n" unless(defined $$c1[0] && defined $$c2[0]);

  my ($x,$y,$z)=map { ($$c1[$_]+$$c2[$_])/2 } (0 .. 2);

  # OpenGL frame (?) : X back, Y right, Z up (x~depth, y~width, z~height)
  my ($depth,$width,$height)=map { ($$c1[$_]-$$c2[$_]) } (0 .. 2);

  my $rgb=$robot{$body}{geom}{$geom}{rgb} || '#ff00ff';
  my ($r,$g,$b)=map { sprintf("%6.4f",hex($_)/255) } unpack "a2a2a2", substr($rgb,1);

  push @xml, qq(
  <Box body="$body" name="$body$geom" x="$x" y="$y" z="$z" depth="$depth" width="$width" height="$height">
   <Frame base="$base" a="0" b="0" g="0" x="0" y="0" z="0" />
   <Color r="$r" g="$g" b="$b" />
  </Box>);
 }
}

push @xml, qq(
 </geometry>);
 
push @xml, qq(
</Robot>);


print @xml;

exit 0;

sub iterate_through_structure {  # Walks through whole structure 
 my ($ele, $fn_hash, $fn_array)=@_;
 my $r=ref($ele);
 return unless($r);
# print "$ele=$r\n";
 if($r eq 'HASH') {
  foreach my $e (keys %$ele) {
#   print "key=$e\n";
   if($fn_hash) {
    $fn_hash->($ele, $e); # Let the fn deal with the finding 
   }
   iterate_through_structure($$ele{$e}, $fn_hash, $fn_array);
  }
 }
 elsif($r eq 'ARRAY') {
  for(my $i=0; $i<@$ele; $i++) {
#   print "arrayele=$e\n";
   if($fn_array) {
    $fn_array->($ele, $i); # Let the fn deal with it
   }
   iterate_through_structure($$ele[$i], $fn_hash, $fn_array);
  }
 }
}

sub test_exists {
 my ($ref, $str)=@_; # 	($robot, "{$body}{CoG}{position}{world}{xyz}");
 my $current='$hashref';
 my @items=split('}{', $str);
 foreach my $i (@items) {
  $i =~ s/[\{\}]//g;
  unless(ref($ref)) {
   die "$current has surprising item '$i'\n";
  }
  unless(exists $$ref{$i}) {
   die "$current has no element '$i'\n";
   last;
  }
  $current .= "{$i}";
  $ref=$$ref{$i}; # Descend
 }
}

sub multiply_matrix_by_vector3 {
 my ($m,$v)=@_;
 my $u=[0,0,0];
 for(my $i=0;$i<3;$i++) {
  for(my $j=0;$j<3;$j++) {
   $$u[$i]+=$$m[$i][$j]*$$v[$j];
  }
 }
 return($u);
}
sub multiply_matrix_by_matrix3 {
 my ($m1, $m2)=@_;
 my $m=[[0,0,0],[0,0,0],[0,0,0]];
 for(my $i=0;$i<3;$i++) {
  for(my $j=0;$j<3;$j++) {
   for(my $k=0;$k<3;$k++) {
    $$m[$i][$j]+=$$m1[$i][$k]*$$m2[$k][$j];
   }
  }
 }
 return($m);
}
sub transpose_matrix3 {
 my ($m1)=@_;
 my $m=[[0,0,0],[0,0,0],[0,0,0]];
 for(my $i=0;$i<3;$i++) {
  for(my $j=0;$j<3;$j++) {
   $$m[$i][$j]=$$m1[$j][$i];
  }
 }
 return($m);
}

