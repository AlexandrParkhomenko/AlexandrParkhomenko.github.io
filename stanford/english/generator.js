/*
  вычитаем из первого списка второй
*/

e = function(__id){
  return document.getElementById(__id)
}

run = function(){
  a = e("textinput1").value.split("\n");
  b = e("textinput2").value.split("\n");
  for(i=0; i<b.length; i++){
    c = a.indexOf(b[i])
console.log("b[i]="+b[i])
console.log("c="+c)
    if(c != -1 )  a.splice(c,1);
console.log(a)
  }
  e("textoutput").value = a.join("\n");
}
