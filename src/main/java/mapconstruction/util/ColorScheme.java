package mapconstruction.util;

import java.awt.*;

/**
 * Enum of color schemes.
 *
 * @author Roel
 */
public enum ColorScheme {

    /**
     * 12 color scale from http://colorbrewer2.org/
     */
    COLOR_BREWER(new String[]{
            "#a6cee3", "#1f78b4", "#b2df8a", "#33a02c", "#fb9a99", "#e31a1c",
            "#fdbf6f", "#ff7f00", "#cab2d6", "#6a3d9a", "#ffff99", "#b15928"
    }),
    /**
     * 100 colors generated from http://tools.medialab.sciences-po.fr/iwanthue/
     * using "Intense" and "Hard".
     */
    COLORS100INTENSE(new String[]{
            "#59B8F4", "#FC1D1D", "#6DCE14", "#DF32E4", "#4C3C04", "#521E52",
            "#E0B218", "#EF1D7C", "#66C58B", "#EEA0C2", "#155561", "#0761CC",
            "#99242B", "#297118", "#EE813F", "#AAA564", "#F974D0", "#7C3A9C",
            "#F88B84", "#25C4D3", "#9DC756", "#456FAC", "#A4A2BA", "#9D6C47",
            "#A0256B", "#29836A", "#D79AF2", "#1EAC46", "#C470FF", "#A08018",
            "#763700", "#1B4919", "#E85249", "#909E01", "#113F8C", "#562F3F",
            "#FE76AB", "#C23058", "#E0AB64", "#756D7E", "#35355D", "#C17800",
            "#5D6314", "#B40086", "#27BEAE", "#9877ED", "#700340", "#A6AAE0",
            "#FE45BB", "#548B02", "#4D2C15", "#1B577C", "#1E523B", "#962F00",
            "#7A1E32", "#C2BC46", "#41D281", "#F3A440", "#EE86FF", "#CA31C1",
            "#3187D8", "#5D48B3", "#1AA664", "#0F6E43", "#C3550E", "#681D0B",
            "#F95031", "#F69EA5", "#432E76", "#363547", "#E18969", "#6A7800",
            "#F7359B", "#EE4F6A", "#C52534", "#F385C0", "#CAA036", "#84C064",
            "#258539", "#A06402", "#A82617", "#68CBC3", "#FFA55A", "#C39BE1",
            "#B350EA", "#4C3154", "#EE92EE", "#158078", "#52A1D5", "#764603",
            "#BA9012", "#ADB6D4", "#62143A", "#7FCF30", "#FF3F81", "#DD4EEE",
            "#428413", "#8F270A", "#5CCF78", "#136384"
    }),
    /**
     * 100 colors generated from http://tools.medialab.sciences-po.fr/iwanthue/
     * using "Pastel" and "Hard".
     */
    COLORS100PASTEL(new String[]{
            "#7FEAF2", "#E99A61", "#B2E787", "#D9A1C6", "#F8EBD0", "#D8C45D", "#93B1D4",
            "#93C69F", "#C8A782", "#FDAFAF", "#F6F7B1", "#8FF4BE", "#77B6BA", "#FEDEE7",
            "#B4BD82", "#BDAAB5", "#A9FFE6", "#B6B6A2", "#D1FFC8", "#C3EDF3", "#FFC578",
            "#F49484", "#D6D6F8", "#FCD19C", "#AEBF66", "#E9FA97", "#EFC0F4", "#5FBDA4",
            "#AFDDFD", "#8DDD92", "#D5B172", "#DCF6D5", "#A1B3BB", "#E3A18F", "#C6B1D1",
            "#BCFDB3", "#CCAEA3", "#70C8DA", "#6BD5CF", "#FEEAB6", "#F6DE80", "#9CBCB2",
            "#A6B58E", "#D8F0E4", "#D1A2B2", "#F7E499", "#CCC36E", "#FCADD0", "#C9C397",
            "#7BF2D8", "#F2DED5", "#A1C790", "#F9AC7D", "#B3C0EB", "#FEBEAE", "#F1C7CA",
            "#ECAC5E", "#DFE4AD", "#FBFED7", "#F1B3BE", "#B8F2D6", "#F9C19A", "#D8EFAA",
            "#72DAAA", "#D2EB7E", "#F0EE94", "#A3BCA4", "#ACC47D", "#80DFCB", "#AEF6BB",
            "#C5F6EB", "#A3DB8D", "#DBA9AC", "#D5A46F", "#AECE6F", "#A2F3A3", "#C6B386",
            "#CDC7F4", "#E6C669", "#6DC6C5", "#B1ACBE", "#EDB861", "#9AF9FA", "#F5D794",
            "#86B4BE", "#80ECC1", "#D2F18D", "#E2F8A0", "#6CB8B5", "#7DF7FB", "#FAE3D0",
            "#81CDD7", "#CD9FBF", "#B5C563", "#FAD4D4", "#92B4D0", "#FEE3B5", "#E4F3D5",
            "#FA9E8D", "#CFA7B2"
    });

    public static final ColorScheme DEFAULT = ColorScheme.COLORS100INTENSE;

    private final String[] hexColors;

    private ColorScheme(String[] hexColors) {
        this.hexColors = hexColors;
    }

    private static Color hexToRGB(String hexcolor) {
        return new Color(
                Integer.parseInt(hexcolor.substring(1), 16)
        );

    }

    /**
     * Gets the i-th color in the default scheme, cyclically.
     *
     * @param i color posiotion, modulo number of colors.
     * @return i-th color (mudula number of colors) in the default scheme
     */
    public static Color getDefaultColorCyclic(int i) {
        return DEFAULT.getColorCyclic(i);
    }

    /**
     * Gets the i-th color in the scheme, cyclically.
     *
     * @param i color position, modulo number of colors.
     * @return i-th color (modulo number of colors) in the scheme
     */
    public Color getColorCyclic(int i) {
        return hexToRGB(hexColors[i % hexColors.length]);
    }

}
