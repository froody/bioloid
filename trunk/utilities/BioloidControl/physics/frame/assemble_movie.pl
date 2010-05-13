#! /usr/bin/perl -w
use strict;

my $output= $ARGV[0] || 'output.mpg';
my @files = map { {orig=>$ARGV[$_], } } (1 .. $#ARGV);
unless(@files>1) {
  print qq(usage : perl assemble_movie.pl <output.mpg> <file1.mpg> <file2.flv> ...
where : <*.flv> will be automatically converted to mpeg as required\n);
  exit 1;
}


# Convert all the files as necessary - noting the temp ones...
foreach my $f (@files) {
  if($$f{orig} =~ m{\.flv$}i) {
    $$f{use}=$$f{orig};
    $$f{use} =~ s{\.flv$}{\.mpg}i;
    
    system qq(ffmpeg -i $$f{orig} -b 1M -r 30 -y $$f{use});
    
    $$f{del} = $$f{use};
  }
  else {
    $$f{use}=$$f{orig};
  }
}

# Now cat them together for the output
my $list=join(' ', map { "'$$_{use}'" } @files);
system qq(cat $list > $output);

# Delete the temporary files
foreach my $f (@files) {
  next unless(exists $$f{del});
  unlink $$f{del};
}

1;

__END__

perl assemble_movie.pl playshow.mpg demo1-title.mpg grab-15a-1M.flv demo-credits.mpg 
perl assemble_movie.pl walking.mpg  demo2-title.mpg walk-15a-1M.flv demo-credits.mpg 
perl assemble_movie.pl balance.mpg  demo3-title.mpg balance-15a-1M.flv demo-credits.mpg 
