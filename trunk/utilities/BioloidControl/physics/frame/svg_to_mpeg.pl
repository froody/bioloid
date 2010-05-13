#! /usr/bin/perl -w
use strict;

my %set=(
  'demo1-title'=>{
    _format=>'title',
   'TopLine'=>'bioloidcontrol.sourceforge.net',
   'MiddleLine1'=>'Physics Simulation',
   'MiddleLine2'=>"Controller 'playshow'",
   'BottomLine'=>'14-Sep-2008',
  },
  'demo2-title'=>{
    _format=>'title',
   'TopLine'=>'bioloidcontrol.sourceforge.net',
   'MiddleLine1'=>'Physics Simulation',
   'MiddleLine2'=>"'test 0.05 5 0 0'",
   'BottomLine'=>'14-Sep-2008',
  },
  'demo3-title'=>{
    _format=>'title',
   'TopLine'=>'bioloidcontrol.sourceforge.net',
   'MiddleLine1'=>'Physics Simulation',
   'MiddleLine2'=>"'perl Bioloid.pl' testing",
   'BottomLine'=>'14-Sep-2008',
  },
  'demo-credits'=>{
    _format=>'credits',
   'TopLine'=>'bioloidcontrol.sourceforge.net',
   'MiddleLine1'=>'Controller : Rainer (cosa)',
   'MiddleLine2'=>'Physics Sim : Martin (mdda)',
   'MiddleLine3'=>'Requires : ODE, C++/Boost',
   'MiddleLine4'=>'Includes : Full robot physics',
   'MiddleLine5'=>'',
   'MiddleLine6'=>"Everything Open Source (GPL2)",
   'BottomLine'=>'Video by mdda',
  },
);

my $set = $ARGV[0] || '';
unless($set && exists $set{$set}) {
  my $sets=join(', ', sort keys %set);
  print qq(usage : perl svg_to_mpeg.pl <set>
where : <set> is from : {$sets}\n);
  exit 1;
}
my %trans=%{$set{$set}};

my $original=$trans{_format} || 'title';
my $updated=$set;

if(open(FIN, "<$original.svg") && open(FOUT, ">$updated.svg")) {
  while(my $l = <FIN>) {
    foreach my $k (keys %trans) {
      my $v=$trans{$k};
      $l =~ s{~$k~}{$v}g;
    }
    print FOUT $l;
  }
  
  close(FIN);
  close(FOUT);
}
else {
  die "Couldn't open the input and/or output files";
}

my $dims=qq(--export-area-canvas --export-width=640 --export-height=480 );
system qq(inkscape -f $updated.svg --export-png=$updated.png $dims);
unlink "$updated.svg";

my $title_time=3; # in secs

my $fps=30;
my $frames=$title_time*$fps;
# -y is overwrite output files
system qq(ffmpeg -loop_input -vframes $frames -i $updated.png -r $fps -y $updated.mpg);
unlink "$updated.png";

1;


# Also : 
# inkscape --verb-list