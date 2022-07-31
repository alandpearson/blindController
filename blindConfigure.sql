-- MySQL dump 10.13  Distrib 5.5.60, for debian-linux-gnu (armv8l)
--
-- Host: localhost    Database: themachine
-- ------------------------------------------------------
-- Server version	5.5.60-0+deb8u1

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!40101 SET NAMES utf8 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

--
-- Table structure for table `blindConfig`
--

DROP TABLE IF EXISTS `blindConfig`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `blindConfig` (
  `nodeId` int(11) DEFAULT NULL,
  `name` varchar(30) DEFAULT NULL,
  `autoOpenMin` int(11) DEFAULT NULL,
  `autoCloseMin` int(11) DEFAULT NULL,
  `sunDipSec` int(11) DEFAULT NULL,
  `sunDipPosition` int(11) DEFAULT NULL,
  `openThreshold` int(11) DEFAULT NULL,
  `closeThreshold` int(11) DEFAULT NULL,
  `sundipThreshold` int(11) DEFAULT NULL,
  `autoWaitMin` int(11) DEFAULT NULL,
  `manWaitMin` int(11) DEFAULT NULL,
  `opMode` int(11) DEFAULT NULL,
  `underRun` int(11) DEFAULT NULL,
  `overRun` int(11) DEFAULT NULL
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `blindConfig`
--

LOCK TABLES `blindConfig` WRITE;
/*!40000 ALTER TABLE `blindConfig` DISABLE KEYS */;
INSERT INTO `blindConfig` VALUES (2,'Lounge',5,5,30,25,500,900,30,10,60,3,3,1),(3,'Master Bedroom',5,5,30,25,500,800,30,10,60,1,3,1),(4,'Snug',5,5,30,25,500,800,28,10,60,3,3,1),(8,'AlOffice',5,5,30,40,500,800,40,10,60,3,3,1),(20,'Breadboard',5,5,30,25,500,800,30,10,60,3,3,1),(9,'Dining',5,5,30,25,500,800,30,10,60,3,3,1);
/*!40000 ALTER TABLE `blindConfig` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2018-08-27  0:17:04
