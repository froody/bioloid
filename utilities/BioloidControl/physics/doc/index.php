<?
    require("html.php");

    $idx="index.php";
    
    /* Comment out the next line to get ordinary behaviour (once this file is copied into index.php) - mdda */
    /* $idx="index_new.php"; */
    
    $page_category = getGETValue("category", 0);
    $page_categories = array(array("Information", "information.php", $idx."?category=0"),
                             array("Robots", "robots.php", $idx."?category=1"),
                             array("", ""),
                             array("Setup", "setup.php", $idx."?category=3"),
                             array("First Start", "firststart.php", $idx."?category=4"),
                             array("", ""),
                             array("Configuration", "configuration.php", $idx."?category=6"),
                             array("Console", "console.php", $idx."?category=7"),
                             array("Scripting Language", "scriptinglanguage.php", $idx."?category=8"),
                             array("", ""),
                             array("Physics Sim :", "", ""),
                             array("&#8226; About", "physics_about.php", $idx."?category=11"),
                             array("&#8226; Building", "physics_building.php", $idx."?category=12"),
                             array("&#8226; Running", "physics_running.php", $idx."?category=13"),
                             array("", "", ""),
                             array("Download", "", "http://sourceforge.net/project/showfiles.php?group_id=225495"),
                             array("Code Documentation", "", "./docs/html/index.html"),
                             array("Project Page", "", "http://sourceforge.net/projects/bioloidcontrol"));

    if ($page_category < 0 || $page_category >= count($page_categories))
       $page_category = 0;
    
    if ($page_categories[$page_category][0] == "" )
       $page_category = 0;
       
    $page_menu = getHTMLMenu($page_categories);
       
    /* Passende Seite für Kategorie laden */
    $page_content = include($page_categories[$page_category][1]);

    $page_footer = include("footer.php");

    $page_html = getHTMLPage($page_menu, $page_content, $page_footer);

    /* Seite an Browser übermitteln/ausgeben */
    //echo utf8_decode($page_html);
    echo $page_html;
?>
