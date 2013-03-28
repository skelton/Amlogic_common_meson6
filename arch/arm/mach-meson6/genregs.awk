#!/usr/bin/awk -f

BEGIN{
	
	
	
}
/^\/\/ Secure APB3 Slot 0 Registers/{
    sec_bus="SECBUS_REG_ADDR"
}
/^\/\/ Secure APB3 Slot 2 Registers/{
    sec_bus="SECBUS2_REG_ADDR"
}

/^#define/ && $2 ~/^P_AO_RTC_/{
#    print $1,$2,"\t\tSECBUS_REG_ADDR(" $8,$9,$10,"\t///" FILENAME
#    ignore RTC
    next
}

/^#define/ && NF==3{
    BASE="CBUS_REG_ADDR";
    if(index(FILENAME,"secure_apb")!=0){
        BASE=sec_bus;
	}else
    if(index(FILENAME,"pctl")!=0 || index(FILENAME,"hdmi")!=0){
        BASE="APB_REG_ADDR";
	}else if( $3 ~ /^0x0?[39ceCE][[:xdigit:]][[:xdigit:]]$|^0x3[fF][[:xdigit:]][[:xdigit:]]$/ )
	{
		BASE="DOS_REG_ADDR";
	}else if( $2 ~ /^VDEC_ASSIST_/)
    {
        BASE="DOS_REG_ADDR";
    }
    if(index(FILENAME,"hdmi")!=0 && $2 ~ /^STIMULUS_/)
        next;
	print $1,$2,$3,"\t///" FILENAME
	print $1,"P_" $2 , "\t\t" BASE "(" $2 ")" ,"\t///" FILENAME
}
/^#define/ && NF==14 && $2 ~ /^P_/{
    print $1,$2,"\t\tAOBUS_REG_ADDR(" $8,$9,$10,$11,$12,$13,$14,"\t///" FILENAME
}
END{
	
}
