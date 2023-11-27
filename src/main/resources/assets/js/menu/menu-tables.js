/**
 * This file contains general functions for the different tables.
 *
 * In this file it is implemented that if you hold shift upon a table press, all the items from the previous table item
 * until this item are selected.
 *
 * Note: The actual implementation of drawing these trajectories is implemented in maps-draw.js
 *
 * @author Jorrick Sleijster
 * @since  26/08/2018
 */


let shifted = false;
$(document).on('keyup keydown', function(e){shifted = e.shiftKey} );

let $lastSelectedItem = '';
function initializeShiftSelectForAllCheckboxes(){
    $('.selector-table .checkbox')
        .checkbox()
        .each(function( ){
            $( this ).checkbox({
                onChange: function() {
                    let $currentSelectedItem = $( this ).closest('tr');

                    if (shifted){
                        let $allNextElements = $currentSelectedItem.nextAll('tr');
                        let $allPrevElements = $currentSelectedItem.prevAll('tr');

                        let foundInNextElements = false;
                        let foundInPrevElements = false;

                        let checkItems = $(this).parent().hasClass('checked');
                        console.log(checkItems);

                        $allNextElements.each(function (){
                            if (this === $lastSelectedItem){
                                foundInNextElements = true;
                            }
                        });

                        $allPrevElements.each(function (){
                            if (this === $lastSelectedItem){
                                foundInPrevElements = true;
                            }
                        });
                        console.log($allNextElements, $allPrevElements);
                        console.log({foundInNextElements, foundInPrevElements});

                        if (foundInNextElements) {
                            $traverseList = $allNextElements;
                        }
                        if (foundInPrevElements){
                            $traverseList = $allPrevElements;
                        }

                        if (foundInNextElements || foundInPrevElements){
                            $traverseList.each(function (){
                                if (checkItems){
                                    $(this).find('.checkbox').checkbox('set checked');
                                    console.log('Checked', $(this));
                                } else {
                                    $(this).find('.checkbox').checkbox('set unchecked');
                                    console.log('Unchecked', $(this));
                                }
                                if (this === $lastSelectedItem){
                                    return false;
                                }
                            });
                        }

                        executeTableDraws();
                    }

                    $lastSelectedItem = $currentSelectedItem.get(0);
                }
            });
        });
}