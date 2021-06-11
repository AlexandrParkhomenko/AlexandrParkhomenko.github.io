/*
  Simple ters generator
*/

e = function(__id){
  return document.getElementById(__id)
}

run = function(){
  a = e("textinput1").value.split("\n");
  let doc = document.createElement('div');
  let div;
  let num = 1;
  for(i=0; i<a.length; i++){
    c = a[i].split(" ");
    if(c[0]==""){
      if(c[1]=="!"){
        c.splice(0, 2); // удаление форматирования
        a[i] = c.join(" ");
        let help = document.createElement('div');
        let text = document.createTextNode(a[i]);
        help.appendChild(text);
        help.className = "help";
        div.appendChild(help);
      } else {
        let t = 0;
        if(c[2]=="=") {
          t = 1;
          c[2] = "-";
          a[i] = c.join(" ");
        }
        let button = document.createElement('button');
        let text = document.createTextNode(a[i]);
        button.appendChild(text);
        button.setAttribute("onclick", "f(this,"+num+","+t+")");
        div.appendChild(button);
      }
    } else {
      if (div != undefined){
        doc.appendChild(div);
        num++;
      }
      let q = "";
      do{
        q = q + a[i] + "\n";
        i++;
      } while(i<a.length && (a[i].substr(0,3).match(/\s[A-Z]\s/)===null))
      i--; // выскочили на ответы, откатываемся
      div = document.createElement('div');
      let pre = document.createElement('pre');
      let text = document.createTextNode(q);
      pre.appendChild(text);
      div.appendChild(pre);
    }
  }
//console.log(a)
  e("textoutput").value = doc.innerHTML;
}
