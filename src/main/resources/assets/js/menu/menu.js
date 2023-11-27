/**
 * This file contains all the logic of general stuff which multiple menu tabs need.
 * This means it handles selecting all elements in a table, switching between menu pages.
 *
 * @author Jorrick Sleijster
 * @since  26/08/2018
 */
function initializeMenu(){
    $('.menu-button').on("click", function(){
        openNewPage($(this) );
    });

    $('.left.sidebar').resizable({
        handles: "e",
        minWidth: 400,
        maxWidth: 1000
    });
}

/**
 * Function for changing the menu item
 *
 * This function is called when a menu item is pressed. It closes all pages(always just one)
 * and opens up the page required by finding the right div based on the menu items text.
 *
 * @param pageItem      jQuery          Menu item that was pressed.
 */
function openNewPage(pageItem){
    let page = pageItem.text();
    page = page.toLowerCase().replace(/\s/g,'');

    $('.sidebar-content').addClass('hidden-content');
    $('#sidebar-'+page).removeClass('hidden-content');

    $('.menu-button').removeClass('selected-menu');
    pageItem.addClass('selected-menu');
}

/**
 * Called by buttons to select everything of the table.
 * @param button
 */
function selectAllOfTable(button){
    doSelectAllOfTable(true, button);
}

/**
 * Called by buttons to deselect everything of the table.
 * @param button
 */
function deselectAllOfTable(button){
    doSelectAllOfTable(false, button);
}

/**
 *
 * @param doSelect boolean, whether we should select everything (if false, we deselect everything)
 * @param button that was pressed
 */
function doSelectAllOfTable(doSelect, button){
    $button = $(button);
    $buttonsContainer = $button.parent();
    $menuPage = $buttonsContainer.parent();
    $table = $($menuPage.find('table')[0]);

    $table.find($('.ui.checkbox')).each(function () {
        if(doSelect){
            $(this).checkbox('check');
        } else {
            $(this).checkbox('uncheck');
        }
    });

    if ($table.hasClass('trajectories-selector-table')){
        removeTableDrawnTrajectories();
        drawAllTrajectoriesFromTheTable();

    }

    if ($table.hasClass('bundles-selector-table')){
        removeTableDrawnBundles();
        drawAllBundlesFromTheTable();
    }

    if ($table.hasClass('intersection-selector-table')){
        removeTableDrawnIntersections();
        drawAllIntersectionsFromTheTable();
    }

    if ($table.hasClass('connections-selector-table')){
        removeTableDrawnConnections();
        drawAllConnectionsFromTheTable();
    }

    if ($table.hasClass('road-edges-selector-table')){
        removeTableDrawnRoads();
        drawAllRoadsFromTheTable();
    }
}