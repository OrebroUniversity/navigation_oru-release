function v = OP_SUBASSIGN()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = casadiMEX(0, 79);
  end
  v = vInitialized;
end
