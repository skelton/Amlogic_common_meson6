#!/usr/bin/awk -f

BEGIN{
	
	
	
}
/^#define/ && NF==3{
    print $1,$2,$3,"\t///" FILENAME
	BASE="CBUS_REG_ADDR";
    if(index(FILENAME,"pctl")!=0){
        BASE="APBBUS_REG_ADDR";
	}if( $3 ~ /^0x0[39ce]/ )
	{
		BASE="DOSBUS_REG_ADDR";
	}
	print $1,"P_" $2 , "\t\t" BASE "(" $2 ")" ,"\t///" FILENAME
}
/^#define/ && NF==14{
    print $1,$2,"\t\tAOBUS_REG_ADDR(" $8,$9,$10,$11,$12,$13,$14,"\t///" FILENAME
}
/^#define/ && NF==10{
    print $1,$2,"\t\tSECBUS_REG_ADDR(" $8,$9,$10,"\t///" FILENAME
}
END{
	
}
