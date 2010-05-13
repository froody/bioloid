#! /bin/perl
use strict;

# cat ~/oss/Bioloid/Simloid-v1.50-mdda/src/build/Bioloid.cpp | perl simloid_C_to_delimited.pl > simloid_C_to_delimited.txt

local $/=undef; # Slurp in whole file
my $contents=<STDIN>;


$contents =~ s{//[^\*]{3}.*?\n}{\n}g;       #  Take out comments to end of line (unless they are like //***)
$contents =~ s{(//\*\*\*.*?)\n}{$1-EOL-}g;  #  preserve the //*** comments to end of line
$contents =~ s{\s+}{ }g;  # Take out all multi-space
$contents =~ s{-EOL-}{;\n}g;
my @rows=split(m{[;\}\{]\s+}, $contents);

foreach my $r (@rows) {
	next if($r =~ m{accel}i); # Ignore all the accel board lines
	next if($r =~ m{hinge|joint}i); # Ignore all the Hinge lines
	
	my @c=split(m{[\(\),]\s*}, $r);
	my $out=join('|',@c);
	if($c[0] =~ m{prepareGeomBody}) {
  print "//Key : Action|name|world|space|obj|obj#|geo#|x|y|z|sx|sy|sz|m|mass|RGB|coll\n";
  $out="$c[0]||$c[1]||$c[2]|$c[3]||$c[4]|$c[5]|$c[6]||||$c[7]";
	}
	if($c[0] =~ m{finalizeGeomBody}) {
		$out="$c[0]||||$c[1]|$c[2]||$c[3]|$c[4]|$c[5]||||$c[6]";
	}
	if($c[0] =~ m{attachGeomBox}) {
		$out="$c[0]|$c[1]||$c[2]|$c[3]|$c[4]|$c[5]|$c[6]|$c[7]|$c[8]|$c[9]|$c[10]|$c[11]|$c[12]|$c[13]|$c[14]|$c[15]|$c[16]|$c[17]";
	}
	if($c[0] =~ m{createBox}) {
		$out="$c[0]|$c[1]|$c[2]|$c[3]|$c[4]|$c[5]||$c[6]|$c[7]|$c[8]|$c[9]|$c[10]|$c[11]|$c[12]||$c[13]|$c[14]|$c[15]|$c[16]";
	}
	
	print $out,"\n";
	
	last if($r =~ m{printStatistics}); # Only interested in first definition of robot
}

__END__
void Primitives::prepareGeomBody(	
         //
									const dWorldID &world,
									//									
									Common::MyObject *obj, const int objnr, 
									//
									const float posx, const float posy, const float posz, 
									//
									dMass &m)
void Primitives::finalizeGeomBody(
         //
         //
         //
         Common::MyObject *obj, const int objnr, 
         //
         //
         //
									dMass &m)
									
void Primitives::attachGeomBox(	
        const char* name,
								//
								const dSpaceID &space,
								Common::MyObject *obj,const int objnr, 
								const int geomnr, 
								const float posx, const float posy, const float posz,
								const float lenx, const float leny, const float lenz,
								dMass &m, const float mass,
								const float colr, const float colg, const float colb,
								bool collision)
								
void Primitives::createBox(	
       const char* name,
							const dWorldID &world,
							const dSpaceID &space,
							Common::MyObject *obj, const int objnr,
							//
							const dReal posx, const dReal posy, const dReal posz,
							const dReal lenx, const dReal leny, const dReal lenz,
							const dReal mass, //
							const dReal colr, const dReal colg, const dReal colb,
							bool collision )



void Primitives::createJoint( 	const dWorldID &world,
								Common::MyObject *obj,
								dJointID *joint,
								const int jointnr,
								const int body1, const int body2,
								const int relBody, const float relx, const float rely, const float relz,
								const char axis )
