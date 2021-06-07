score = 0;
  a = new Array();
  
function f(o,b,c){
  if(a.indexOf(b)==-1){
    if (c==1){
      o.style.background = "#aaffaa";
    } else {
      o.style.background = "#ffaaaa";
    }
    if (c==1) score++;
  a.push(b)
  }
  document.getElementById("top").innerHTML = "score:"+score+"/"+a.length
}
