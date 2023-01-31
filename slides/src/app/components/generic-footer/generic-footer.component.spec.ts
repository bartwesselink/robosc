import { ComponentFixture, TestBed } from '@angular/core/testing';

import { GenericFooterComponent } from './generic-footer.component';

describe('GenericFooterComponent', () => {
  let component: GenericFooterComponent;
  let fixture: ComponentFixture<GenericFooterComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ GenericFooterComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(GenericFooterComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
