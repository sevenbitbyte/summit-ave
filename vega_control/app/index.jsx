// You can also include here commons if you want with import 'react-toolbox/lib/commons';

// https://facebook.github.io/react/docs/two-way-binding-helpers.html

import React from 'react';
import ReactDOM from 'react-dom';
import ToolboxApp from 'react-toolbox/lib/app';
import LayoutTest from './components/layout_test';
//import LeftMenu from './components/left_menu';
import style from './style';


ReactDOM.render((
  <ToolboxApp>
    <LayoutTest />
  </ToolboxApp>
), document.getElementById('app'));
