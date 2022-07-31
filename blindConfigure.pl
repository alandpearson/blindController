#!/usr/bin/perl -w

#Configure blindControllers from database, sending via RF using mqtt to emonpi

use DBI;
use Net::MQTT::Simple;
use Net::MQTT::Simple::Auth;
use POSIX  'strftime';
use Config::Simple;

use File::Basename qw(dirname);
use Cwd  qw(abs_path);
use lib dirname(dirname abs_path $0) . '/libs/perl';

use HomeAutomation ;


my $cfg = new Config::Simple('blindConfigure.cfg') ;

#Database params
my $dbName = $cfg->param('mysql_db');
my $dbUser = $cfg->param('mysql_user');
my $dbPass = $cfg->param('mysql_pass');
my $dbHost = $cfg->param('mysql_db_hostname');

#MQTT params
my $mqttTopic = $cfg->param('mqtt_topic') ;

my $dbh = DBI->connect("DBI:mysql:$dbName:$dbHost", $dbUser, $dbPass);



sub menu {


	my $nodeId = 0 ;

	print "Blind Controller configuration programme\n\n";
	print "Found the following blind controllers in the database \n";
        print "\tnodeId: name\n";


	my $sth = $dbh->prepare("SELECT nodeId,name  FROM blindConfig");
	$sth->execute();

         while (@data = $sth->fetchrow_array()) {
	    $nodeId = $data[0];
            my $name = $data[1];
            print "\t$nodeId: $name\n";
          }

          if ($sth->rows == 0) {
            print "No blinds found in the database !\n\n";	
	    $nodeId = 0;
          } else {

		print "Enter nodeId you want to configure\n";
		$nodeId = <STDIN> ;
		chomp $nodeId ;

	  }

          $sth->finish;


	return $nodeId ;


}

sub mqttConfigMsg {

	my $nodeId = $_[0] ;

	print "nodeId" . $nodeId ;

}


sub printNodeConfig {
	
	my $nodeId = $_[0] ;
	my $mqttValues = "0" ;

	my $sth = $dbh->prepare("SELECT * FROM blindConfig where nodeId= $nodeId");
	$sth->execute();

          if ($sth->rows == 0) {
            print "Configuration not found for node $nodeId in the database !\n\n";	
	    $mqttValues = "0";
          } else {

		print "\n*****************************\n";
		print "Configuration for nodeId " . $nodeId . "\n" ;
		$ref = $sth->fetchrow_hashref ;

		$mqttValues="4," . 
			$ref->{'opMode'} . "," .
			$ref->{'nodeId'} . "," .
			$ref->{'autoOpenMin'} . "," .
			$ref->{'autoCloseMin'} . "," .
			$ref->{'sunDipSec'} . "," .
			$ref->{'sunDipPosition'} . "," .
			$ref->{'openThreshold'} . "," .
			$ref->{'closeThreshold'} . "," .
			$ref->{'sundipThreshold'} . "," .
			$ref->{'autoWaitMin'} . "," .
			$ref->{'manWaitMin'} . "," .
			$ref->{'opMode'} . "," .
			$ref->{'underRun'} . "," .  
			$ref->{'overRun'} ; 
			print "$mqttValues \n";

		foreach $key (keys %$ref)  {
		    print "${key}: " ;
		    print $ref->{$key},"\n"; 
		}

		print "*****************************\n";

		print "Proceed with sending configuration to node $nodeId ? \n";
		my $response = <STDIN> ;
		chomp $response ;
		if ( $response eq 'y' || $response eq 'Y') {
			print "Confirmed.\n";
			 #cmd,opmode,nodeid,autoOpenMin,autoCloseMin,sunDipSec,sunDipPosition,openThreshold,closeThreshold,sundipThreshold,autoWaitMin,manWaitMin,opMode,underRun,overRun,crc

		} else {
			#Cancelled
			$mqttValues = "1";
		}

	  }

	return $mqttValues ;


}




#static word calcCrc (const void* ptr, byte len) {
#  word crc = ~0;
#  for (byte $i = 0; $i < len; ++i)
#    crc = _crc16_update(crc, ((const byte*) ptr)[i]);
#  return crc;
#}



sub crc16_update {
   
	my ($crc, $a) = @_ ; 
	my $i = 0 ;

    $crc ^= $a;
    for ($i = 0; $i < 8; ++$i)
    {
        if ($crc & 1) {
            $crc = ($crc >> 1) ^ 0xA001;
       } else {
            $crc = ($crc >> 1);
	}
    }

    return $crc;
}



sub calcCrc {

	my $mqttMsg = $_[0] ;


	my $crc = 65535;
	my @values = split(',', $mqttMsg) ;
	print "$mqttMsg \n" ;
	$data = pack("CCCSCSSSCCCCC", $values[2], $values[3], $values[4], $values[5], $values[6] , $values[7], $values[8], $values[9], $values[10], $values[11], $values[12], $values[13], $values[14]);

	for ($i=0 ; $i < 17 ;  ++$i) {
		$crc = crc16_update($crc, vec $data,$i,8 ) ;
	}

	return ($crc) 
}





#########################################################################################################################
# Main															#
#########################################################################################################################

my $mqtt ; 
my $nodeId = 0 ;
my $mqttMsg = "0" ;
my %mqttData = () ;

# MQTT Connect 
if ($cfg->param('mqtt_enabled') eq "true") {
        print ("MQTT enabled \n");
        $mqtt = Net::MQTT::Simple::Auth->new($cfg->param('mqtt_hostname'), $cfg->param('mqtt_user'), $cfg->param('mqtt_pass') );
        if ( $mqtt ) {
                print ("MQTT connected to broker " . $cfg->param('mqtt_hostname') . "\n") ;
        } else {
                print ("MQTT ERROR could not connect to broker " . $cfg->param('mqtt_hostname') . "\n") ;
        }

} else {
        print ("MQTT disabled by configuration \n");
}



# Let's go ....

#Present menu of nodes in database and get user to choose one
$nodeId = menu() ;

# Retrieve the configuration and put into mqtt message
$mqttData{msg} = printNodeConfig($nodeId) ;

# Set MQTT topic to emonhub/tx/<nodeid>/values  for TX by emonpi
$mqttTopic =  $mqttTopic . "/$nodeId/values" ;

# If we were able to create the mqtt value string send it
if ($mqttData{msg} eq "0" ) {
	print "ERROR creating configuration - exiting\n" ;
} elsif ($mqttData{msg} eq "1" ) {
	print "Cancelled.\n";
} else {
	# Add the crc 
	$mqttData{msg} .= "," . calcCrc($mqttData{msg}) ;
	HomeAutomation::mqttSend(\$mqtt, $mqttTopic, %mqttData) ;
	$mqtt->tick(5);
	print "Configuration sent to blind - it will move slightly up and down if received OK\n" ;
} 

sleep (2) ;
$dbh->disconnect;
exit () ;
