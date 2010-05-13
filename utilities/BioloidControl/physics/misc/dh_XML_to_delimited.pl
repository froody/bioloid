#! /bin/perl
use strict;

# cat ../../main/dh_humanoid.xml | perl dh_XML_to_delimited.pl > dh_XML_to_delimited.txt

# local $/=undef; # Slurp in whole file
# Line-by-line is fine
while(my $r=<STDIN>) {
	next if($r =~ m{collision}i);  # These lines mess us up
	
	$r =~ s{[\r\n]}{}g;             # Clean off line-endings
 $r =~ s{^\s+}{}g;              # Trim start of line
	next unless($r);

 $r =~ s{\s+(\S+)\=}{\|$1\|}g;  # Put in lots of separators on ids
#	my @c=split(m{[\s\=]+}, $r);  # Just put in separators
	my @c=split(m{\|}, $r);        # Now split it up
	my @c=map { s{\"}{}g; $_; } @c;# Clean out the quoted numbers
	my $out=join('|', @c); 

 if($r =~ m{Frame}) {
#  print "//Key : Action|name|world|space|obj|obj#|geo#|x|y|z|sx|sy|sz|m|mass|RGB|coll\n";
  next;  # Actually, just ignore
  $out="$out";
	}
	if($r =~ m{Color}) {
  next;  # Actually, just ignore
  $out="||$out";
	}
#	if($c[0] =~ m{attachGeomBox}) {
#		$out="$c[0]|$c[1]||$c[2]|$c[3]|$c[4]|$c[5]|$c[6]|$c[7]|$c[8]|$c[9]|$c[10]|$c[11]|$c[12]|$c[13]|$c[14]|$c[15]|$c[16]|$c[17]";
#	}
#	if($c[0] =~ m{createBox}) {
#		$out="$c[0]|$c[1]|$c[2]|$c[3]|$c[4]|$c[5]||$c[6]|$c[7]|$c[8]|$c[9]|$c[10]|$c[11]|$c[12]||$c[13]|$c[14]|$c[15]|$c[16]";
#	}
	
	print $out,"\n";
	
	last if($r =~ m{printStatistics}); # Only interested in first definition of robot
}

__END__
		<chain name= "legl" base="" len="7">
			<dh rotz="0" 		transz="0" rotx="0" 	transx="32.5" id="0" sgn="1" />
                <Box name="Brust" width="100.8" depth="50.4" height="42" x="-32.5" y="13" z="52.5" mass="0">
                        <Frame base="legl:0" a="0" b="0" g="0" x="0" y="0" z="0" />
                        <Color r="0.07" g="0.07" b="0.07" />
		<Collision first="Linke Hand" second="Brust,Huefte links,Huefte rechts,Linker Oberschenkel,Linkes Knie,Linker Unterschenkel,Linker Knoechel,Linker Fuss,Linke Fussplatte,Oberkoerper,Huefte,Huefte,Rechter Oberschenkel,Rechtes Knie,Rechter Unterschenkel,Rechter Knoechel,Rechter Fuss,Rechte Fussplatte,Rechte Schulter,Rechter Oberarm,Rechter Unterarm,Kopf,Rechte Hand,Rechter Oberarm 2" />
