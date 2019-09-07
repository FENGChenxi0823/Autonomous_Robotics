/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Heap Sort Stencil | JavaScript support functions

    Quick JavaScript Code-by-Example Tutorial 
     
    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License 

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/


// create empty object 
minheaper = {}; 

// define insert function for min binary heap
function minheap_insert(heap, new_element) {

    // STENCIL: implement your min binary heap insert operation
    // heap[i] = new_element;
    heap.push(new_element);
    
    var n = heap.length - 1;
    while(n > 0){
      var parentN = Math.floor((n+1)/2)-1;
      parent = heap[parentN];
      if(new_element >= parent)
        break;
      heap[parentN] = new_element;
      heap[n] = parent;
      n = parentN;
    }

}

// assign insert function within minheaper object
minheaper.insert = minheap_insert;
/* Note: because the minheap_insert function is an object, we can assign 
      a reference to the function within the minheap object, which can be called
      as minheap.insert
*/

// define extract function for min binary heap
function minheap_extract(heap) {

    // STENCIL: implement your min binary heap extract operation
    var min = heap[0];
    var n = heap.length - 1;
    var end = heap.pop();

    if(heap.length > 0){
        heap[0] = end;
        var i = 0;
        while (true){
          var l = 2*i + 1;
          var r = 2*i + 2;
          var smallest = i;
          if(l < heap.length && heap[l] < heap[i]){
              smallest = l;
          }
          if(r < heap.length && heap[r] < heap[smallest]){
              smallest = r;
          }
          if(smallest != i){
              var s = heap[smallest];
              heap[smallest] = heap[i];
              heap[i] = s;
              i = smallest;
          }else break;
        }
    }

    return min;

}


// assign extract function within minheaper object

    // STENCIL: ensure extract method is within minheaper object
minheaper.extract = minheap_extract;






