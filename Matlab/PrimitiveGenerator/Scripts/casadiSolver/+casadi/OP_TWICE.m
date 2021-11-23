function v = OP_TWICE()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 23);
  end
  v = vInitialized;
end
