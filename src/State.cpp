<!DOCTYPE html >
 < html >
 < head >
 < meta charset = "utf-8" / >
                  < title > paste.unixcube.org - Paste #e9de6a</ title>
                  < link rel = "stylesheet" href = "/res/document.css" >
                                                   < link rel =
                                                       "stylesheet" href =
                                                           "/res/markdown.css" >
                                                           <
                                                           link
                                                           rel =
                                                               "stylesheet" href
= "/res/googlecode.css" >

                                                                                   <
                                                                                   script
                                                                                   src
                                                                                       =
"/res/highlight/highlight.pack.js" > < / script >

                                                                                         <
                                                                                         link
                                                                                         rel
                                                                                             =
"stylesheet" href = "/res/highlight/styles/github.css" >

                                                                                                                   <
                                                                                                                   script
                                                                                                                   src
                                                                                                                       =
"/res/prism/prism.js" > < / script >
                                                                                                                         <
                                                                                                                         link
                                                                                                                         rel
                                                                                                                             =
"stylesheet" href = "/res/prism/prism.css" >



                                                                                                                                                   <
                                                                                                                                                   /
                                                                                                                                                   head
                                                                                                                                                   >

                                                                                                                                                   <
                                                                                                                                                   body
                                                                                                                                                   >

                                                                                                                                                   <
                                                                                                                                                   div
                                                                                                                                                   id
                                                                                                                                                       =
"header" >
                                                                                                                                                         <
                                                                                                                                                         h2
                                                                                                                                                         >
                                                                                                                                                         <
a href = "/" > paste.unixcube.org</ a> < / h2 >
                                                                                                                                                                    <
                                                                                                                                                                    /
                                                                                                                                                                    div
                                                                                                                                                                    >

                                                                                                                                                                    <
                                                                                                                                                                    div
                                                                                                                                                                    id
                                                                                                                                                                        =
"info" >
                                                                                                                                                                          <
                                                                                                                                                                          div
                                                                                                                                                                          id
                                                                                                                                                                              =
"infotext" >
                                                                                                                                                                                <
                                                                                                                                                                                h6
                                                                                                                                                                                >
                                                                                                                                                                                15
                                                                                                                                                                                lines
                                                                                                                                                                                <
                                                                                                                                                                              br
/>
                                                                                                                                                                                Pasted
                                                                                                                                                                                at
                                                                                                                                                                                2016
                                                                                                                                                                                -
02 - 08 14 : 06 : 10 - 0800 PST
                                                                                                                                                                                <
                                                                                                                                                                              /
                                                                                                                                                                              h6>
                                                                                                                                                                                <
                                                                                                                                                                                /
                                                                                                                                                                                div
                                                                                                                                                                                >

                                                                                                                                                                                <
                                                                                                                                                                                div
                                                                                                                                                                                id
                                                                                                                                                                                    =
"links" >
                                                                                                                                                                                      <
                                                                                                                                                                                      a
                                                                                                                                                                                      href
                                                                                                                                                                                          =
"/t/e9de6a" id = "link-a" > Raw Text</ a>
                                                                                                                                                                                                             <
                                                                                                                                                                                                             input
                                                                                                                                                                                                             id
                                                                                                                                                                                                                 =
"textbox" type = "text" value =
                                                                                                                                                                                                                                        "https://paste.unixcube.org/k/e9de6a"
                                                                                                                                                                                                                                        /
                                                                                                                                                                                                                                        >
                                                                                                                                                                                                                                        <
                                                                                                                                                                                                                                        /
                                                                                                                                                                                                                                        div
                                                                                                                                                                                                                                        >
                                                                                                                                                                                                                                        <
                                                                                                                                                                                                                                        /
                                                                                                                                                                                                                                        div
                                                                                                                                                                                                                                        >

                                                                                                                                                                                                                                        <
                                                                                                                                                                                                                                        pre
                                                                                                                                                                                                                                        id
                                                                                                                                                                                                                                            =
"codepre" > < span id = "linenumbers" > 1
                                                                                                                                                                                                                                                                      2
                                                                                                                                                                                                                                                                      3
                                                                                                                                                                                                                                                                      4
                                                                                                                                                                                                                                                                      5
                                                                                                                                                                                                                                                                      6
                                                                                                                                                                                                                                                                      7
                                                                                                                                                                                                                                                                      8
                                                                                                                                                                                                                                                                      9
                                                                                                                                                                                                                                                                      10
                                                                                                                                                                                                                                                                      11
                                                                                                                                                                                                                                                                      12
                                                                                                                                                                                                                                                                      13
                                                                                                                                                                                                                                                                      14
                                                                                                                                                                                                                                                                      15
                                                                                                                                                                                                                                                                      <
                                                                                                                                                                                                                                                                      /
                                                                                                                                                                                                                                                                      span
                                                                                                                                                                                                                                                                      >
< code id = "codetag" >   // =============================================================================
// File Name: State.cpp
// Description: Defines State in StateMachine class
// Author: FRC Team 3512, Spartatroniks
// =============================================================================

#include &#34; State.hpp &#34;

                                                                                                                                                                                                                                                                                    State
                                                                                                                                                                                                                                                                                    ::
                                                                                                                                                                                                                                                                                    State(
                                                                                                                                                                                                                                                                          std
::string name) : m_name{std::move(name)} {
 }

 const std::string& amp; State::Name() const {
     return m_name;
 }

 < / code > < / pre >

 < h6 id = "expiration" >
           Paste expires 2016 - 03 - 09 14 : 06 : 10 - 0800 PST</ h6>


           < script type = "text/javascript" >
                           function highlight() {
     languageLookupTable =
     {'objectivec':'objectivec', 'ini':'ini', 'bash':'bash', 'go':'go',
      'xml':'markup', 'perl':'perl', 'markdown':'markdown', 'python':'python',
      'cs':'csharp', 'http':'http', 'sql':'sql', 'apache':'apacheconf',
      'php':'php', 'java':'java', 'cpp':'cpp', 'javascript':'javascript',
      'ruby':'ruby', 'css':'css', 'coffeescript':'coffeescript',
      'makefile':'makefile'};

     var text = document.getElementById('codetag');
     result = hljs.highlightAuto(text.textContent);

     language = languageLookupTable[result.language];

     if (language != undefined) {
         highlighted =
             Prism.highlight(text.textContent, Prism.languages[language]);
         text.innerHTML = highlighted;
     }
     else {
         text.innerHTML = result.value;
     }
 }

 if (window.addEventListener) {
     window.addEventListener('load', highlight, false);
 }
 else {
     window.attachEvent('onload', highlight);
 }




 < / script >


 < / body >
 < / html>
